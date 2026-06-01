// 小红点设置，小红点
/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (Successfully Restored to Old-Version Speed & Curve)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * 终极轴对称、纯物理几何加速、彻底根治久用敏感度偏斜版
 * Copyright (c) 2026 ZitaoTech & Gemini
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

/* ========= TrackPoint 专用 Work Queue ========= */
#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 5

static struct k_mutex trackpoint_i2c_mutex;

K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

/* ========================================================================= */
/* 参数映射区 (100% 物理对称兼容版)                                           */
/* ========================================================================= */
#define SCROLL_X_DIR (-CONFIG_TRACKPOINT_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_TRACKPOINT_SCROLL_Y_DIR

#define MOUSE_BASE_SPEED (CONFIG_TRACKPOINT_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_TRACKPOINT_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_TRACKPOINT_MOUSE_SENS_STEP_PERCENT / 100.0f)

/* ========= Motion GPIO ========= */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_I2C_ADDR 0x15
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

#define SLOW_KEY_MULTIPLIER 0.3f
#define FAST_KEY_MULTIPLIER 2.5f

/* ========= Watch Dog ========= */
static uint32_t last_activity_time = 0;
#define TRACKPOINT_WDT_TIMEOUT 200

/* ========= 全局状态 ========= */
static bool scroll_key_pressed = false;
static bool arrow_key_pressed = false;
static bool slow_key_pressed = false;
static bool fast_key_pressed = false; // 👈 🟢 新增：快门键按下状态
static bool last_scroll_key_pressed = false; 
static bool last_arrow_key_pressed = false;

// 🔴 彻底删除了旧版的全局 uint32_t last_packet_time，杜绝命名空间污染！

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

/* ========= 按键监听 ========= */
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
    
    if (ev->position == 22) {
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
    uint32_t last_packet_time; // 🟢 整个系统唯一的、受结构体保护的时间戳
    int16_t scroll_residue_x; 
    int16_t scroll_residue_y;
    int16_t arrow_residue_x;
    int16_t arrow_residue_y;
};

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
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    // 🟢 鼠标移动专用的静态残留变量
    static float mouse_rem_x = 0.0f;
    static float mouse_rem_y = 0.0f;
    static int8_t last_sign_x = 0;
    static int8_t last_sign_y = 0;

    // 🟢 滚轮模式专用的高精度浮点残留变量
    static float scroll_rem_x = 0.0f;
    static float scroll_rem_y = 0.0f;

    /* ========= WATCHDOG 喂狗修复 ========= */
    if (now - last_activity_time > TRACKPOINT_WDT_TIMEOUT) {
        LOG_WRN("TrackPoint watchdog recovery");
        last_activity_time = now;
        data->last_packet_time = now;
        last_scroll_key_pressed = scroll_key_pressed;
        last_arrow_key_pressed = arrow_key_pressed;
        
        // 静止唤醒时彻底洗刷全部模式的残留，确保重回绝对零位对称
        mouse_rem_x = 0.0f; mouse_rem_y = 0.0f; last_sign_x = 0; last_sign_y = 0;
        scroll_rem_x = 0.0f; scroll_rem_y = 0.0f;
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
    /* 1. 方向键模式                                                            */
    /* ========================================================================= */
    if (arrow_key_pressed) {
        // 跨模式清洗，防止小数残留造成污染
        mouse_rem_x = 0.0f; mouse_rem_y = 0.0f; last_sign_x = 0; last_sign_y = 0;
        scroll_rem_x = 0.0f; scroll_rem_y = 0.0f;

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
    /* 2. 滚轮模式 —— ⭐ 同步重构为绝对物理力矩加速，彻底废除 powf 和时间抖动污染 */
    /* ========================================================================= */
    else if (scroll_key_pressed || capslock) {
        // 跨模式清洗
        mouse_rem_x = 0.0f; mouse_rem_y = 0.0f; last_sign_x = 0; last_sign_y = 0;

        if (dx != 0 || dy != 0) {
            float scroll_exp_mult = 1.0f;
            float physical_dist = sqrtf((float)(dx * dx + dy * dy));
            
            if (physical_dist > 1.0f) {
                // 采用几何多项式无感加速，替代旧版极其不稳定的时间戳 powf 算法
                scroll_exp_mult = 1.0f + (physical_dist * 0.15f);
                if (scroll_exp_mult > 5.0f) scroll_exp_mult = 5.0f;
            }

            // 完美的各向异性高精度映射
            float fx_scroll = ((float)dx * SCROLL_X_DIR * scroll_exp_mult) / 48.0f;
            float fy_scroll = ((float)dy * SCROLL_Y_DIR * scroll_exp_mult) / 16.0f;

            scroll_rem_x += fx_scroll;
            scroll_rem_y += fy_scroll;

            int16_t scroll_x = (int16_t)scroll_rem_x;
            int16_t scroll_y = (int16_t)scroll_rem_y;

            scroll_rem_x -= scroll_x;
            scroll_rem_y -= scroll_y;

            if (scroll_x != 0 || scroll_y != 0) {
                input_report_rel(dev, INPUT_REL_HWHEEL, scroll_x, false, K_NO_WAIT);
                input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_NO_WAIT);
            }

            k_sleep(K_MSEC(5)); 
        }
    }

 /* ========================================================================= */
    /* 3. 正常鼠标移动模式 —— ⭐ 独家调校：高容错率电竞级微操曲线 (驯服突然暴冲与直线性) */
    /* ========================================================================= */
    else {
        // 释放滚轮残留
        scroll_rem_x = 0.0f; scroll_rem_y = 0.0f;

        uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
        float tp_factor = MOUSE_SENS_BASE + MOUSE_SENS_STEP * tp_led_brt;

        int8_t cur_dx = dx;
        int8_t cur_dy = dy;

        // 方向逆转秒清零
        int8_t sign_x = (cur_dx > 0) ? 1 : ((cur_dx < 0) ? -1 : 0);
        int8_t sign_y = (cur_dy > 0) ? 1 : ((cur_dy < 0) ? -1 : 0);
        
        if (sign_x != 0 && sign_x != last_sign_x) { mouse_rem_x = 0.0f; last_sign_x = sign_x; }
        if (sign_y != 0 && sign_y != last_sign_y) { mouse_rem_y = 0.0f; last_sign_y = sign_y; }

        if (cur_dx == 0 && cur_dy == 0) {
            mouse_rem_x = 0.0f;
            mouse_rem_y = 0.0f;
            last_sign_x = 0;
            last_sign_y = 0;
        } else {
            // 🟢 核心重构：平滑S型物理加速曲线
            float exp_mult = 1.0f;
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
            float physical_dist = sqrtf((float)(cur_dx * cur_dx + cur_dy * cur_dy));
            
            if (physical_dist > 1.0f) {
                // 1. 将原先陡峭的 0.15f 降为极度细腻的 0.04f，平抑微推时的敏感度
                // 2. 引入平滑多项式，使速度变化率连续，彻底消灭“突变临界点”
                float delta_dist = physical_dist - 1.0f;
                exp_mult = 1.0f + (delta_dist * 0.04f) + (delta_dist * delta_dist * 0.003f);
                
                // 限制最高增益上限
                if (exp_mult > 2.8f) exp_mult = 2.8f; 
            }
#endif

            float slow_mult = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;
            float fast_mult = fast_key_pressed ? FAST_KEY_MULTIPLIER : 1.0f;

            // 🟢 调校基础倍率（将原本生硬的 2.5f 调整为更可控的 1.8f 基准，配合上面的平滑多项式）
            // 这样能让鼠标在低速时有极高的操控感，而当你大力推时，通过多项式依然能飞起来
            float fx = (float)cur_dx * 1.8f * MOUSE_BASE_SPEED * tp_factor * exp_mult * slow_mult;
            float fy = (float)cur_dy * 1.8f * MOUSE_BASE_SPEED * tp_factor * exp_mult * slow_mult;

            // 高精度累加
            mouse_rem_x += (-fx);
            mouse_rem_y += (-fy);
        }

        int final_x = (int)mouse_rem_x;
        int final_y = (int)mouse_rem_y;

        mouse_rem_x -= final_x;
        mouse_rem_y -= final_y;

        if (final_x != 0 || final_y != 0) {
            input_report_rel(dev, INPUT_REL_X, final_x, false, K_NO_WAIT);
            input_report_rel(dev, INPUT_REL_Y, final_y, true, K_NO_WAIT); 
        }
    }

    // 🟢 维护唯一合法的结构体内部时间戳
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

    LOG_INF("TrackPoint Driver Initialized (Pure Physics Symmetric Mode)");
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
