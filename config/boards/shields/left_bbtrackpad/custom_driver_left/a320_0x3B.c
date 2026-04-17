/*
 * A320 optical sensor driver (polling via motion GPIO, Zephyr input subsystem)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT avago_a320

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <stdlib.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include "trackpad_led.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

/* ==== HID indicators ==== */
static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1 << 1)

/* ==== Motion GPIO ==== */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 5
static const struct device *motion_gpio_dev;

/* ==== Touch flag ==== */
static bool touched = false;

/* =========================
 *   Data & Config structs
 * ========================= */
struct a320_dev_config {
    struct i2c_dt_spec i2c;
};

typedef int (*a320_read_fn_t)(const struct device *dev, int16_t *dx, int16_t *dy);

struct a320_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
    a320_read_fn_t read_motion;
};

/* =========================
 *   Ctrl key listener
 * ========================= */
// static bool ctrl_pressed = false;
// static bool space_pressed = false; // 我尝试添加的。

/*
static int ctrl_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) {
        return 0;
    }

    if (ev->position == 37) {
        ctrl_pressed = ev->state;
    }
    return 0;
}

ZMK_LISTENER(a320_ctrl_listener, ctrl_listener_cb);
ZMK_SUBSCRIPTION(a320_ctrl_listener, zmk_position_state_changed);
*/ 
// 上面这段我注释掉了。
//从这里
/* =========================
 * Key State Listener (Ctrl & Space)
 * ========================= */
static bool ctrl_pressed = false;
static bool space_pressed = false;

static int a320_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) {
        return 0;
    }

    // 处理 Ctrl 键 (位置 37) - 用于降速，这个排序他估计搞错了，但是ctrl不好用，我改成f。但这个命名我就不改了，就这样吧。之前不是ctrl，其实是l。
    if (ev->position == 30 ) {
        ctrl_pressed = ev->state;
    }

    // 处理 Space 键 (位置 60) - 用于触发滚动
    if (ev->position == 60) {
        space_pressed = ev->state;
        // LOG_INF 可以帮你调试，确认位置对不对
        LOG_INF("Space %s", space_pressed ? "HELD" : "RELEASED");
    }

    return 0;
}

// 定义一个通用的监听器名
ZMK_LISTENER(a320_key_listener, a320_key_listener_cb);
// 订阅位置状态改变事件
ZMK_SUBSCRIPTION(a320_key_listener, zmk_position_state_changed);
// 到这里

/* =========================
 *   HID indicator listener
 * ========================= */
static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev) {
        current_indicators = ev->indicators;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

/* =========================
 *   I2C read variants
 * ========================= */

/* ---- 0x3B variant ---- */
static int a320_read_motion_3b(const struct device *dev, int16_t *dx, int16_t *dy) {
    const struct a320_dev_config *cfg = dev->config;
    uint8_t buf[3];
    uint8_t reg = 0x82;

    if (i2c_write_dt(&cfg->i2c, &reg, 1) < 0) {
        return -EIO;
    }

    if (i2c_burst_read_dt(&cfg->i2c, 0x82, buf, sizeof(buf)) < 0) {
        return -EIO;
    }

    *dx = (int8_t)buf[1];
    *dy = -(int8_t)buf[2];
    return 0;
}

/* ---- 0x37 variant ---- */
static int a320_read_motion_37(const struct device *dev, int16_t *dx, int16_t *dy) {
    const struct a320_dev_config *cfg = dev->config;
    uint8_t buf[7];
    uint8_t reg = 0x0A;

    if (i2c_write_dt(&cfg->i2c, &reg, 1) < 0) {
        return -EIO;
    }

    if (i2c_burst_read_dt(&cfg->i2c, 0x0A, buf, sizeof(buf)) < 0) {
        return -EIO;
    }

    *dy = -(int8_t)buf[1];
    *dx = -(int8_t)buf[3];
    return 0;
}

/* =========================
 *   Poll work
 * ========================= */
#ifndef CONFIG_A320_POLL_INTERVAL_MS
#define CONFIG_A320_POLL_INTERVAL_MS 2
#endif

static void a320_poll_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, poll_work);

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    if (pin_state == 0) {
        int16_t dx = 0, dy = 0;

        if (data->read_motion(data->dev, &dx, &dy) == 0 && (dx || dy)) {
            // bool space_pressed = current_indicators & HID_INDICATORS_CAPS_LOCK;

            if (ctrl_pressed) {
                dx /= 2;
                dy /= 2; // 原先是2，暂时来看，足够了。
            }

            if (!space_pressed) {
                uint8_t brt = indicator_tp_get_last_valid_brightness();
                float factor = 0.4f + 0.01f * brt;
                dx = dx * 3 / 2 * factor;
                dy = dy * 3 / 2 * factor;
            }

            if (space_pressed) {
                input_report_rel(data->dev, INPUT_REL_WHEEL, -dy / 16, true, K_FOREVER);
            } else {
                input_report_rel(data->dev, INPUT_REL_X, dx, false, K_FOREVER);
                input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_FOREVER);
                touched = true;
            }
        }
    } else {
        touched = false;
    }

    k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
}

bool tp_is_touched(void) { return touched; }

/* =========================
 *   Init
 * ========================= */
static int a320_init(const struct device *dev) {
    const struct a320_dev_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    if (!device_is_ready(cfg->i2c.bus)) {
        return -ENODEV;
    }

    motion_gpio_dev = DEVICE_DT_GET(MOTION_GPIO_NODE);
    if (!device_is_ready(motion_gpio_dev)) {
        return -ENODEV;
    }
    gpio_pin_configure(motion_gpio_dev, MOTION_GPIO_PIN, GPIO_INPUT | GPIO_PULL_UP);

    /* Select read function by I2C address */
    switch (cfg->i2c.addr) {
    case 0x3B:
        data->read_motion = a320_read_motion_3b;
        break;
    case 0x37:
        data->read_motion = a320_read_motion_37;
        break;
    default:
        LOG_ERR("Unsupported A320 addr: 0x%02X", cfg->i2c.addr);
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init_delayable(&data->poll_work, a320_poll_work_handler);
    k_work_schedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));

    LOG_INF("A320 init OK addr=0x%02X", cfg->i2c.addr);
    return 0;
}

/* =========================
 *   Device define
 * ========================= */
#define A320_INIT_PRIORITY CONFIG_INPUT_A320_INIT_PRIORITY

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_dev_config a320_cfg_##inst = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_cfg_##inst, POST_KERNEL, \
                          A320_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)

ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);
