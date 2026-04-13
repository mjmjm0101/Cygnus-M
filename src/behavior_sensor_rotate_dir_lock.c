/*
 * Sensor rotate behavior with direction lock.
 * Prevents direction reversal for CONFIG_ZMK_BEHAVIOR_SENSOR_ROTATE_DIR_LOCK_HOLD_TIME_MS
 * after the last rotation event.
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_rotate_dir_lock

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/sensors.h>
#include <zmk/behavior_queue.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_sensor_rotate_dir_lock_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    int tap_ms;
};

struct behavior_sensor_rotate_dir_lock_data {
    struct sensor_value remainder[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int triggers[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int8_t last_direction[ZMK_KEYMAP_SENSORS_LEN];    /* 1=CW, -1=CCW, 0=unknown */
    int64_t last_event_time_ms[ZMK_KEYMAP_SENSORS_LEN];
};

static int sensor_rotate_dir_lock_accept_data(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
    const struct zmk_sensor_channel_data *channel_data) {

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_rotate_dir_lock_data *data = dev->data;

    const struct sensor_value value = channel_data[0].value;
    int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    int triggers;

    if (value.val1 == 0) {
        triggers = value.val2;
    } else {
        struct sensor_value remainder = data->remainder[sensor_index][event.layer];

        remainder.val1 += value.val1;
        remainder.val2 += value.val2;

        if (remainder.val2 >= 1000000 || remainder.val2 <= -1000000) {
            remainder.val1 += remainder.val2 / 1000000;
            remainder.val2 %= 1000000;
        }

        int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
        triggers = remainder.val1 / trigger_degrees;
        remainder.val1 %= trigger_degrees;

        data->remainder[sensor_index][event.layer] = remainder;
    }

    /* Direction lock logic */
    if (triggers != 0) {
        int8_t direction = (triggers > 0) ? 1 : -1;
        int64_t now = k_uptime_get();

        if (data->last_direction[sensor_index] != 0 &&
            direction != data->last_direction[sensor_index] &&
            (now - data->last_event_time_ms[sensor_index]) <
                CONFIG_ZMK_BEHAVIOR_SENSOR_ROTATE_DIR_LOCK_HOLD_TIME_MS) {
            /* クールダウン中の逆方向入力を破棄 */
            triggers = 0;
        } else {
            data->last_direction[sensor_index] = direction;
        }

        data->last_event_time_ms[sensor_index] = now;
    }

    data->triggers[sensor_index][event.layer] = triggers;
    return 0;
}

static int sensor_rotate_dir_lock_process(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    enum behavior_sensor_binding_process_mode mode) {

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_dir_lock_config *cfg = dev->config;
    struct behavior_sensor_rotate_dir_lock_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][event.layer];

    struct zmk_behavior_binding triggered_binding;
    if (triggers > 0) {
        triggered_binding = cfg->cw_binding;
        triggered_binding.param1 = binding->param1;
    } else if (triggers < 0) {
        triggers = -triggers;
        triggered_binding = cfg->ccw_binding;
        triggered_binding.param1 = binding->param2;
    } else {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    for (int i = 0; i < triggers; i++) {
        zmk_behavior_queue_add(&event, triggered_binding, true, cfg->tap_ms);
        zmk_behavior_queue_add(&event, triggered_binding, false, 0);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_sensor_rotate_dir_lock_api = {
    .sensor_binding_accept_data = sensor_rotate_dir_lock_accept_data,
    .sensor_binding_process     = sensor_rotate_dir_lock_process,
};

#define SENSOR_ROTATE_DIR_LOCK_INST(n)                                                        \
    static struct behavior_sensor_rotate_dir_lock_config                                      \
        behavior_sensor_rotate_dir_lock_config_##n = {                                        \
            .cw_binding  = {.behavior_dev =                                                   \
                                DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0))},     \
            .ccw_binding = {.behavior_dev =                                                   \
                                DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1))},     \
            .tap_ms = DT_INST_PROP(n, tap_ms),                                               \
        };                                                                                    \
    static struct behavior_sensor_rotate_dir_lock_data                                        \
        behavior_sensor_rotate_dir_lock_data_##n = {0};                                      \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL,                                                    \
                            &behavior_sensor_rotate_dir_lock_data_##n,                        \
                            &behavior_sensor_rotate_dir_lock_config_##n,                      \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                 \
                            &behavior_sensor_rotate_dir_lock_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ROTATE_DIR_LOCK_INST)