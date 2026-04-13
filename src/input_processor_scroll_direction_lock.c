#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#define DT_DRV_COMPAT zmk_input_processor_scroll_direction_lock

#ifndef CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY
#define CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY 50
#endif

struct scroll_dir_lock_data {
    int8_t last_direction;      /* 1 = positive, -1 = negative, 0 = unknown */
    int64_t last_event_time_ms;
};

static int scroll_dir_lock_handle_event(const struct device *dev,
                                        struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct scroll_dir_lock_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return 0;
    }
    if (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL) {
        return 0;
    }
    if (event->value == 0) {
        return 0;
    }

    int8_t direction = event->value > 0 ? 1 : -1;
    int64_t now = k_uptime_get();

    if (data->last_direction != 0 &&
        direction != data->last_direction &&
        (now - data->last_event_time_ms) <
            CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_DIR_LOCK_HOLD_TIME_MS) {
        /* クールダウン中に逆方向が来たら前方向に上書き */
        event->value = (int32_t)(data->last_direction * abs(event->value));
    } else {
        data->last_direction = direction;
    }

    data->last_event_time_ms = now;
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_dir_lock_api = {
    .handle_event = scroll_dir_lock_handle_event,
};

#define SCROLL_DIR_LOCK_INST_INIT(inst)                                     \
    static struct scroll_dir_lock_data scroll_dir_lock_data_##inst = {0};  \
    DEVICE_DT_INST_DEFINE(inst,                                             \
                          NULL,                                             \
                          NULL,                                             \
                          &scroll_dir_lock_data_##inst,                    \
                          NULL,                                             \
                          POST_KERNEL,                                      \
                          CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY,        \
                          &scroll_dir_lock_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_DIR_LOCK_INST_INIT)