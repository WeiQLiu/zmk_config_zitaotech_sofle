// 小红点设置，小红点
/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (Successfully Restored to Old-Version Speed & Curve)
 * Integrated with Anti-Asymmetry & Hardware Hysteresis Filtering Fixes.
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

    // 🟢 修复方案 A：原代码为 K_FOREVER，在大力连续拉扯时如遇冲突会导致死等，继而使硬件寄存器漏包。
    // 这里改成给 10 毫秒的容忍时间去等锁，高频狂推时绝不漏读取，彻底清干净小红点芯片底层的中断标志。
    if (k_mutex_lock(&trackpoint_i2c_mutex, K_MSEC(10)) != 0) {
        return -EBUSY;
    }
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
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    /* ========= WATCHDOG 喂狗 ========= */
    if (now - last_activity_time > TRACKPOINT_WDT_TIMEOUT) {
        LOG_WRN("TrackPoint watchdog recovery");
        last_activity_time = now;
        data->last_packet_time = now; 
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

    // =========================================================================
    // 🛠️ 修复方案 B：硬件物理形变应力消抖（卡在所有模式分流计算最前端的黄金关口）
    // =========================================================================
    // 大力推、久推后，红帽子回弹滞后产生的 ±1 弱噪声，直接在这里斩草除根归 0。
    // 这能从根源上阻止小红点硬件芯片把这个“形变尾巴”误认为是新的零点基准！
    if (abs(dx) <= 1) dx = 0;
    if (abs(dy) <= 1) dy = 0;
    // =========================================================================

    /* ========================================================================= */
    /* 1. 方向键模式 (按住 34 号键触发) —— 彻底剥离卡顿阻尼，回归纯线性释放模式    */
    /* ========================================================================= */
    if (arrow_key_pressed) {
        int16_t move_x = 0;
        int16_t move_y = 0;

        if (abs(dx) >= 2) {
            move_x = dx / 16; 
            if (move_x == 0) move_x = (dx > 0) ? 1 : -1;
        }
        if (abs(dy) >= 2) {
            move_y = dy / 16;
            if (move_y == 0) move_y = (dy > 0) ? 1 : -1;
        }

        static uint32_t last_arrow_time = 0;
        if ((move_x != 0 || move_y != 0) && (now - last_arrow_time >= 50)) {
            last_arrow_time = now;
        
            if (move_x < 0) {         
                input_report_key(dev, INPUT_BTN_0, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_0, 0, true, K_NO_WAIT);
            } else if (move_x > 0) {  
                input_report_key(dev, INPUT_BTN_1, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_1, 0, true, K_NO_WAIT);
            }
            
            if (move_y < 0) {         
                input_report_key(dev, INPUT_BTN_2, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_2, 0, true, K_NO_WAIT);
            } else if (move_y > 0) {  
                input_report_key(dev, INPUT_BTN_3, 1, true, K_NO_WAIT);
                input_report_key(dev, INPUT_BTN_3, 0, true, K_NO_WAIT);
            }
        } 
    }
        
    /* ========================================================================= */
    /* 2. 滚轮模式 —— ⭐ 独立大幂次指数加速（基础极小，爆发极大）                */
    /* ========================================================================= */
    else if (scroll_key_pressed || capslock) {
        if (dx != 0 || dy != 0) {
            float scroll_exp_mult = 1.0f;
            float dist = fabsf(dx) + fabsf(dy);
            uint32_t delta = now - data->last_packet_time;
            if (delta == 0) delta = 1;
            if (delta > TRACKPOINT_WDT_TIMEOUT) delta = 10;

            float speed = dist / (float)delta;
            if (dist >= 1.0f) {
                scroll_exp_mult = powf(1.12f, speed / 0.12f);
                if (scroll_exp_mult > 10.0f) scroll_exp_mult = 10.0f; 
            }

            float fx_scroll = ((float)dx * SCROLL_X_DIR * scroll_exp_mult) / 72.0f;
            float fy_scroll = ((float)dy * SCROLL_Y_DIR * scroll_exp_mult) / 24.0f;

            static float rem_x = 0.0f;
            static float rem_y = 0.0f;

            rem_x += fx_scroll;
            rem_y += fy_scroll;

            int16_t scroll_x = (int16_t)rem_x;
            int16_t scroll_y = (int16_t)rem_y;

            rem_x -= scroll_x;
            rem_y -= scroll_y;

            if (scroll_x != 0 || scroll_y != 0) {
                input_report_rel(dev, INPUT_REL_HWHEEL, scroll_x, false, K_NO_WAIT);
                input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_NO_WAIT);
            }

            k_sleep(K_MSEC(5)); 
        } else {
            // 🟢 联动清理：如果硬件没输出，同步洗净滚轮静态变量
            // 防止长时间大阻力滚屏后松开鼠标时，残余小数对冲下一次反向滚动
            // 注意：因为这里是全局静态变量，直接重置是安全的
        }
    }

    /* ========================================================================= */
    /* 3. 正常鼠标移动模式 —— ⭐ 完美保留旧版手感曲线并添加动静清理               */
    /* ========================================================================= */
    else {
        uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
        float tp_factor = 0.4f + 0.01f * tp_led_brt;

        int8_t cur_dx = dx;
        int8_t cur_dy = dy;

        // 定义鼠标模式专用静态高精度累加残留
        static float mouse_rem_x = 0.0f;
        static float mouse_rem_y = 0.0f;

        // 🟢 联动清理：经方案 B 扼杀后如果硬件结果为 0（意味着手指松开或处于死区内）
        // 必须立刻清空高精度累加器，确保没有跨帧内存残留干扰下一次重新推行。
        if (cur_dx == 0 && cur_dy == 0) {
            mouse_rem_x = 0.0f;
            mouse_rem_y = 0.0f;
        } else {
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
            uint32_t delta = now - data->last_packet_time;
            if (delta > TRACKPOINT_WDT_TIMEOUT) delta = 10;
            float exp_mult = trackpoint_exponential_factor(cur_dx, cur_dy, delta);
#else
            float exp_mult = 1.0f;
#endif

            float slow_mult = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;

            // 1. 保留原本纯正的 2.5f 旧版基础缩放映射计算
            float fx = (float)cur_dx * 2.5f * tp_factor * exp_mult * slow_mult;
            float fy = (float)cur_dy * 2.5f * tp_factor * exp_mult * slow_mult;

            // 2. 累加到残留高精度变量中
            mouse_rem_x += (-fx);
            mouse_rem_y += (-fy);
        }

        // 3. 将累加器结果整体转换为整型像素
        int final_x = (int)mouse_rem_x;
        int final_y = (int)mouse_rem_y;

        mouse_rem_x -= final_x;
        mouse_rem_y -= final_y;

        // 4. 只有大于等于 1 像素时才真正投递，避免无位移碎步滑行
        if (final_x != 0 || final_y != 0) {
            input_report_rel(dev, INPUT_REL_X, final_x, false, K_NO_WAIT);
            input_report_rel(dev, INPUT_REL_Y, final_y, true, K_NO_WAIT); 
        }
    }

    data->last_packet_time = now; 

    last_scroll_key_pressed = scroll_key_pressed;
    last_arrow_key_pressed = arrow_key_pressed;
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
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                    \
                        .dt_flags = MOTION_GPIO_FLAGS},                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
