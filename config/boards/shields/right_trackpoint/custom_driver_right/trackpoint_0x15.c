// 小红点设置，小红点
/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (Successfully Restored to Old-Version Speed & Curve)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* ========= ⭐ TrackPoint 专用 Work Queue ========= */
#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 5

static struct k_mutex trackpoint_i2c_mutex;

K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

/* ========================================================================= */
/* 鼠标与滚轮可调参数 (保持Kconfig映射，防止编译报错，但核心算法已回归旧版) */
/* ========================================================================= */
#define SCROLL_X_DIR (-CONFIG_TRACKPOINT_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_TRACKPOINT_SCROLL_Y_DIR

/* ========= ⭐ 完美复刻：旧版本专属指数加速参数 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_EXP_BASE 1.04f
#define TP_SPEED_SCALE 0.14f
#define TP_MAX_MULT 3.0f
#endif

/* ========= Motion GPIO ========= */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_I2C_ADDR 0x15
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

#define SLOW_KEY_MULTIPLIER 0.5f

/* ========= Watch Dog ========= */
static uint32_t last_activity_time = 0;
#define TRACKPOINT_WDT_TIMEOUT 200

/* ========= 全局状态 ========= */
static bool scroll_key_pressed = false;
static bool arrow_key_pressed = false;
static bool slow_key_pressed = false;
static bool last_scroll_key_pressed = false; 
static bool last_arrow_key_pressed = false;

/* ==== HID indicators ==== */
static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1 << 1)

static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev) {
        current_indicators = ev->indicators;
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);

/* ========= 按键监听：34(Arrow), 36(Slow), 61(Space) ========= */
static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev)
        return 0;

    if (ev->position == 34) {
        arrow_key_pressed = ev->state;
        // LOG_INF("arrow position=34 %s", arrow_key_pressed ? "PRESSED" : "RELEASED");
        LOG_INF("Arrow Mode Key (pos=%d) %s", ev->position, arrow_key_pressed ? "PRESSED" : "RELEASED");
    }

    if (ev->position == 61) {
        scroll_key_pressed = ev->state;
        LOG_INF("scroll position=61 %s", scroll_key_pressed ? "PRESSED" : "RELEASED");
    }

    if (ev->position == 36) {
        slow_key_pressed = ev->state;
        LOG_INF("slow_key position=36 %s", slow_key_pressed ? "PRESSED" : "RELEASED");
    }

    return 0;
}
ZMK_LISTENER(trackpoint_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_special_key_listener, zmk_position_state_changed);

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work; 
    uint32_t last_packet_time;
    int16_t scroll_residue_x; // 留空保持结构体兼容，代码中不用
    int16_t scroll_residue_y;
    int16_t arrow_residue_x;
    int16_t arrow_residue_y;
};

/* ========= ⭐ 100% 旧版本纯正 powf 指数加速算法 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
static inline float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0) {
        delta_ms = 1;
    }

    float dist = fabsf(dx) + fabsf(dy);
    if (dist < 1.0f) {
        return 1.0f;
    }

    float speed = dist / (float)delta_ms;
    float mult = powf(TP_EXP_BASE, speed / TP_SPEED_SCALE);

    if (mult > TP_MAX_MULT) {
        mult = TP_MAX_MULT;
    }

    return mult;
}
#endif

/* ========= 读取数据包 ========= */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};
    int ret;

    k_mutex_lock(&trackpoint_i2c_mutex, K_FOREVER);
    ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);
    k_mutex_unlock(&trackpoint_i2c_mutex);

    if (ret < 0)
        return ret;

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0)
        return -EIO;

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* ========= 核心工作队列回调函数 ========= */
/* ========= 核心工作队列回调函数 ========= */
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    /* ========= WATCHDOG 喂狗 ========= */
    if (now - last_activity_time > TRACKPOINT_WDT_TIMEOUT) {
        LOG_WRN("TrackPoint watchdog recovery");
        last_activity_time = now;
        last_scroll_key_pressed = scroll_key_pressed;
        last_arrow_key_pressed = arrow_key_pressed;
        return;
    }

    int8_t dx = 0, dy = 0;
    int ret = trackpoint_read_packet(dev, &dx, &dy);
    if (ret != 0) {
        LOG_WRN("TrackPoint I2C read failed (soft recover)");
        return;
    }

    last_activity_time = now;
    bool capslock = current_indicators & HID_INDICATORS_CAPS_LOCK;

    /* ========================================================================= */
    /* 1. 方向键模式 (按住 34 号键触发) —— 彻底剥离卡顿阻尼，回归纯线性释放模式    */
    /* ========================================================================= */
    /* 1. 方向键模式 (按住 34 号键触发) —— 彻底剥离卡顿阻尼，回归纯线性释放模式   */
    /* ========================================================================= */
    if (arrow_key_pressed) {
        int16_t move_x = 0;
        int16_t move_y = 0;

        // 直接采用老版本无方向轴锁定的纯线性逻辑
        if (abs(dx) >= 2) {
            move_x = dx / 16; 
            if (move_x == 0) move_x = (dx > 0) ? 1 : -1;
        }
        if (abs(dy) >= 2) {
            move_y = dy / 16;
            if (move_y == 0) move_y = (dy > 0) ? 1 : -1;
        }

        // 用时间戳控制发包节奏（每 50ms 吐一次按键脉冲），既不阻塞总线，又能让系统完美识别
        static uint32_t last_arrow_time = 0;
        if ((move_x != 0 || move_y != 0) && (now - last_arrow_time >= 50)) {
            last_arrow_time = now;
        
            // 🟢 方向修正：根据你反馈的“全颠倒”，将大于0和小于0对应的虚拟键码互换
            if (move_x < 0) {         // 改变判定方向
                input_report_key(dev, INPUT_BTN_0, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_0, 0, true, K_NO_WAIT);
            } else if (move_x > 0) {  // 改变判定方向
                input_report_key(dev, INPUT_BTN_1, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_1, 0, true, K_NO_WAIT);
            }
            
            if (move_y < 0) {         // 改变判定方向
                input_report_key(dev, INPUT_BTN_2, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_2, 0, true, K_NO_WAIT);
            } else if (move_y > 0) {  // 改变判定方向
                input_report_key(dev, INPUT_BTN_3, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_3, 0, true, K_NO_WAIT);
            }
        } 
    }
        
    /* ========================================================================= */
    /* 2. 滚轮模式 —— ⭐ 滚轮专属高动态指数加速曲线（轻推极慢，重推刷屏）        */
    /* ========================================================================= */
    else if (scroll_key_pressed || capslock) {
        int16_t scroll_x = 0;
        int16_t scroll_y = 0;

        // 只要硬件有读数，就立刻无缝进入处理逻辑
        if (dx != 0 || dy != 0) {
            
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
            uint32_t delta = now - data->last_packet_time;
            if (delta > TRACKPOINT_WDT_TIMEOUT) {
                delta = 10;
            }
            // 1. 🟢 允许所有读数（包括 ±1）全部参与指数加速计算，实现绝对无缝过渡
            float scroll_exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
            float scroll_exp_mult = 1.0f;
#endif

            // 2. 🟢 大幅度减小分母（从 48/24 缩减到 8.0f 和 4.0f），释放极速刷屏的潜力
            // 当你大力推时，scroll_exp_mult 暴涨到 3.0，分子变大，速度将呈指数级飞快提升！
            float fx_scroll = ((float)dx * SCROLL_X_DIR) / 8.0f * scroll_exp_mult; 
            float fy_scroll = ((float)dy * SCROLL_Y_DIR) / 4.0f * scroll_exp_mult;

            // 3. 🟢 使用真正的截断或四舍五入，让真正的物理浮点数决定是否发包
            scroll_x = (int16_t)roundf(fx_scroll);
            scroll_y = (int16_t)roundf(fy_scroll);

            // 4. 🟢 保底补偿：只有当 fx_scroll 算出来确实连 0.5 都不到（导致 roundf 变成 0）
            // 但手指又确实在推时，才给 1 或 -1 的微动，确保绝对不丢包、能精准微调
            if (dx != 0 && scroll_x == 0) scroll_x = (dx * SCROLL_X_DIR > 0) ? 1 : -1;
            if (dy != 0 && scroll_y == 0) scroll_y = (dy * SCROLL_Y_DIR > 0) ? 1 : -1;

            // 呈报给系统
            input_report_rel(dev, INPUT_REL_HWHEEL, scroll_x, false, K_NO_WAIT);
            input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_NO_WAIT);

            // 5. 🟢 降低防抖睡眠：从 30ms 缩短到 10ms
            // 中断模式下，把睡眠时间留长了会严重限制大推时的发包频率，导致大力推时“快不起来”
            k_sleep(K_MSEC(10)); 
        }
    }



        
    /* ========================================================================= */
    /* 3. 正常鼠标移动模式 —— ⭐ 精准修复版（彻底解决方向不对称与起飞卡顿）    */
    /* ========================================================================= */
    else {
        uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
        float tp_factor = 0.4f + 0.01f * tp_led_brt;

        // 引入软件消抖，但不设硬死区
        int8_t cur_dx = dx;
        int8_t cur_dy = dy;

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
        uint32_t delta = now - data->last_packet_time;
        if (delta > TRACKPOINT_WDT_TIMEOUT) delta = 10;
        float exp_mult = trackpoint_exponential_factor(cur_dx, cur_dy, delta);
#else
        float exp_mult = 1.0f;
#endif

        float slow_mult = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;

        // 1. 同步计算完美的浮点数矢量坐标
        float fx = (float)cur_dx * 2.5f * tp_factor * exp_mult * slow_mult;
        float fy = (float)cur_dy * 2.5f * tp_factor * exp_mult * slow_mult;

        // 2. 使用更顺滑的 roundf，且【不要】在这里单独对单轴做过于生硬的 1 像素保底
        int final_x = (int)roundf(-fx);
        int final_y = (int)roundf(-fy);

        // 如果真的有移动，哪怕很微弱，保证方向正确的微动，同时投递
        if (cur_dx != 0 && final_x == 0) final_x = (cur_dx > 0) ? -1 : 1;
        if (cur_dy != 0 && final_y == 0) final_y = (cur_dy > 0) ? -1 : 1;

        // 3. ⭐ 核心同步：先上报 X，【绝不】在中间插任何逻辑或延迟，紧接着上报 Y 并投递同步信号
        input_report_rel(dev, INPUT_REL_X, final_x, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, final_y, true, K_NO_WAIT); // 真正的合流投递
    }
    // 5. 时间戳安全位置：确保每一次数据处理（无论走哪个分支）时间步长都在连续更新
    data->last_packet_time = now; 

    last_scroll_key_pressed = scroll_key_pressed;
    last_arrow_key_pressed = arrow_key_pressed;
    // 删掉原本在最后的 data->last_packet_time = now; 避免重复更新
}



/* ========= GPIO 中断接收服务 ========= */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);
    last_activity_time = k_uptime_get_32();
    
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static void trackpoint_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, enable_irq_work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    LOG_INF("TrackPoint IRQ enabled (delayed)");
}

/* ========= 初始化函数 ========= */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;
    if (!i2c_is_ready_dt(&cfg->i2c))
        return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio))
        return -ENODEV;

    k_mutex_init(&trackpoint_i2c_mutex);

    data->dev = dev;
    data->scroll_residue_x = 0;
    data->scroll_residue_y = 0;
    data->arrow_residue_x = 0;
    data->arrow_residue_y = 0;
    data->last_packet_time = k_uptime_get_32();

    k_work_init(&data->work, trackpoint_work_cb);

    k_work_queue_start(&tp_workq, tp_workq_stack, K_THREAD_STACK_SIZEOF(tp_workq_stack),
                       TP_WORKQ_PRIORITY, NULL);

    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);

    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    k_work_init_delayable(&data->enable_irq_work, trackpoint_enable_irq_work_cb);
    k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("TrackPoint Driver Initialized (Hybrid Pure Speed Mode)");
    return 0;
}

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                    \
                        .pin = MOTION_GPIO_PIN,                                                     \
                        .dt_flags = MOTION_GPIO_FLAGS},                                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
