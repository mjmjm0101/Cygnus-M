/*
 * ZMK Input Processor: Scroll Inertia
 *
 * Adds inertial scrolling to trackball scroll mode.
 *
 * The processor monitors scroll velocity and, once a flick is detected,
 * takes over at the moment deceleration begins — suppressing the
 * trackball's natural slowdown and replacing it with a smooth,
 * configurable decay curve.  This eliminates the perceptible gap
 * between the physical scroll stopping and inertia kicking in.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_inertia

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>

#include <drivers/input_processor.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Fixed-point: 8-bit fractional part (×256) */
#define FP_SHIFT 8
#define FP_SCALE (1 << FP_SHIFT)

/* Axis mode constants */
#define AXIS_BOTH 0
#define AXIS_Y    1
#define AXIS_X    2

/* Consecutive EMA decreases required to confirm deceleration */
#define DECEL_CONFIRM_COUNT 2

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

struct scroll_inertia_config {
    int32_t gain;
    int32_t blend;

    int32_t start_fp;
    int32_t move;
    int32_t release_ms;

    int32_t fast_fp;
    int32_t decay_fast;
    int32_t decay_slow;
    int32_t stop_fp;

    int32_t scale;
    int32_t scale_div;
    int32_t limit_fp;
    int32_t span_ms;
    int32_t tick_ms;

    int32_t axis;
    int32_t cancel;
    int32_t layer;
};

/* ------------------------------------------------------------------ */
/* Runtime state                                                       */
/* ------------------------------------------------------------------ */

struct scroll_inertia_data {
    const struct device *dev;

    /* Velocity EMA (fixed-point ×256) */
    int32_t vel_x;
    int32_t vel_y;

    /* Peak velocity in current gesture (fixed-point) */
    int32_t peak_vel_x;
    int32_t peak_vel_y;

    /* Cumulative |delta| for the current gesture */
    int32_t total_movement;

    /* Consecutive EMA-decrease count for deceleration detection */
    int32_t decel_count;

    /* Sub-unit scroll accumulators */
    int32_t accum_x;
    int32_t accum_y;

    /* Inertia state */
    bool inertia_active;
    int64_t inertia_start_time;

    /* Delayed work items */
    struct k_work_delayable stop_detect_work;
    struct k_work_delayable inertia_tick_work;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static inline int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

static inline int32_t clamp_velocity(int32_t vel, int32_t limit_fp) {
    if (vel > limit_fp)  return limit_fp;
    if (vel < -limit_fp) return -limit_fp;
    return vel;
}

static void reset_gesture(struct scroll_inertia_data *data) {
    data->vel_x = 0;
    data->vel_y = 0;
    data->peak_vel_x = 0;
    data->peak_vel_y = 0;
    data->total_movement = 0;
    data->decel_count = 0;
}

static void cancel_inertia(struct scroll_inertia_data *data) {
    data->inertia_active = false;
    k_work_cancel_delayable(&data->inertia_tick_work);
    data->accum_x = 0;
    data->accum_y = 0;
    reset_gesture(data);
}

static void start_inertia(struct scroll_inertia_data *data,
                          const struct scroll_inertia_config *cfg) {
    data->inertia_active = true;
    data->inertia_start_time = k_uptime_get();
    data->accum_x = 0;
    data->accum_y = 0;
    k_work_cancel_delayable(&data->stop_detect_work);

    LOG_DBG("Inertia start  vel_y=%d vel_x=%d  peak_y=%d peak_x=%d  mov=%d",
            data->vel_y, data->vel_x,
            data->peak_vel_y, data->peak_vel_x, data->total_movement);

    k_work_schedule(&data->inertia_tick_work, K_MSEC(cfg->tick_ms));
}

/* ------------------------------------------------------------------ */
/* Inertia tick                                                        */
/* ------------------------------------------------------------------ */

static void inertia_tick_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, inertia_tick_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    if (!data->inertia_active) {
        return;
    }

    /* ── Layer gate ── */
    if (cfg->layer >= 0 && !zmk_keymap_layer_active(cfg->layer)) {
        LOG_DBG("Inertia cancelled: layer %d inactive", cfg->layer);
        cancel_inertia(data);
        return;
    }

    /* ── Duration gate ── */
    if (k_uptime_get() - data->inertia_start_time > cfg->span_ms) {
        cancel_inertia(data);
        return;
    }

    /* ── Two-stage decay ── */
    if (cfg->axis != AXIS_X) {
        int32_t d = abs32(data->vel_y) > cfg->fast_fp
                        ? cfg->decay_fast : cfg->decay_slow;
        data->vel_y = (int64_t)data->vel_y * d / 1000;
    }
    if (cfg->axis != AXIS_Y) {
        int32_t d = abs32(data->vel_x) > cfg->fast_fp
                        ? cfg->decay_fast : cfg->decay_slow;
        data->vel_x = (int64_t)data->vel_x * d / 1000;
    }

    /* ── Stop gate ── */
    bool below_y = (cfg->axis == AXIS_X) || abs32(data->vel_y) < cfg->stop_fp;
    bool below_x = (cfg->axis == AXIS_Y) || abs32(data->vel_x) < cfg->stop_fp;
    if (below_y && below_x) {
        cancel_inertia(data);
        return;
    }

    /* ── Accumulate & emit ── */
    int16_t emit_x = 0, emit_y = 0;

    if (cfg->axis != AXIS_X) {
        data->accum_y += (int64_t)data->vel_y * cfg->scale / cfg->scale_div;
        emit_y = (int16_t)(data->accum_y >> FP_SHIFT);
        data->accum_y -= (int32_t)emit_y << FP_SHIFT;
    }
    if (cfg->axis != AXIS_Y) {
        data->accum_x += (int64_t)data->vel_x * cfg->scale / cfg->scale_div;
        emit_x = (int16_t)(data->accum_x >> FP_SHIFT);
        data->accum_x -= (int32_t)emit_x << FP_SHIFT;
    }

    if (emit_x != 0 || emit_y != 0) {
        zmk_hid_mouse_movement_set(0, 0);
        zmk_hid_mouse_scroll_set(emit_x, emit_y);
        zmk_endpoints_send_mouse_report();
        zmk_hid_mouse_scroll_set(0, 0);
    }

    k_work_schedule(&data->inertia_tick_work, K_MSEC(cfg->tick_ms));
}

/* ------------------------------------------------------------------ */
/* Stop detection (fallback for abrupt stops)                          */
/* ------------------------------------------------------------------ */

static void stop_detect_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, stop_detect_work);
    const struct scroll_inertia_config *cfg = data->dev->config;

    /* Already running from deceleration detection */
    if (data->inertia_active) {
        return;
    }

    bool vel_ok = false;
    if (cfg->axis != AXIS_X) vel_ok |= (abs32(data->vel_y) >= cfg->start_fp);
    if (cfg->axis != AXIS_Y) vel_ok |= (abs32(data->vel_x) >= cfg->start_fp);

    bool mov_ok = data->total_movement >= cfg->move;

    LOG_DBG("Stop detect (fallback): vel_y=%d vel_x=%d mov=%d vel_ok=%d mov_ok=%d",
            data->vel_y, data->vel_x, data->total_movement, vel_ok, mov_ok);

    if (vel_ok && mov_ok) {
        start_inertia(data, cfg);
    } else {
        reset_gesture(data);
    }
}

/* ------------------------------------------------------------------ */
/* Input processor callback                                            */
/* ------------------------------------------------------------------ */

static int scroll_inertia_handle_event(const struct device *dev,
                                       struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    struct scroll_inertia_data *data = dev->data;
    const struct scroll_inertia_config *cfg = dev->config;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool is_y = (event->code == INPUT_REL_WHEEL  && cfg->axis != AXIS_X);
    bool is_x = (event->code == INPUT_REL_HWHEEL && cfg->axis != AXIS_Y);

    if (!is_y && !is_x) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* ────────────────────────────────────────────────────────────────
     * INERTIA ACTIVE — decide whether to suppress or cancel
     * ──────────────────────────────────────────────────────────────── */
    if (data->inertia_active) {
        if (event->value == 0) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* Which axis velocity to compare against? */
        int32_t inertia_vel = is_y ? data->vel_y : data->vel_x;
        int32_t event_vel_fp = (int32_t)event->value << FP_SHIFT;
        bool same_dir = (event->value > 0) == (inertia_vel > 0);

        if (same_dir && abs32(event_vel_fp) <= abs32(inertia_vel)) {
            /* Ball's natural deceleration — suppress this event
             * so only the inertia decay drives scrolling. */
            event->value = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* Reverse direction or faster input — user is re-controlling.
         * Cancel inertia and fall through to normal tracking. */
        LOG_DBG("Inertia cancelled by re-input: val=%d inertia_vel=%d",
                event->value, inertia_vel);
        cancel_inertia(data);
        /* Fall through to tracking below */
    }

    /* ────────────────────────────────────────────────────────────────
     * TRACKING — update velocity and detect deceleration
     * ──────────────────────────────────────────────────────────────── */
    if (event->value == 0) {
        /* Zero events just prove the ball is in motion — reset the
         * stop-detect fallback timer but don't touch velocity. */
        k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Save previous velocity for deceleration detection */
    int32_t prev_vel_y = data->vel_y;
    int32_t prev_vel_x = data->vel_x;

    /* Update EMA */
    if (is_y) {
        int32_t delta_fp = (int32_t)event->value << FP_SHIFT;
        data->vel_y = ((int64_t)delta_fp * cfg->gain +
                       (int64_t)data->vel_y * cfg->blend) / 1000;
        data->vel_y = clamp_velocity(data->vel_y, cfg->limit_fp);
        data->total_movement += abs32(event->value);

        if (abs32(data->vel_y) > abs32(data->peak_vel_y)) {
            data->peak_vel_y = data->vel_y;
        }
    }
    if (is_x) {
        int32_t delta_fp = (int32_t)event->value << FP_SHIFT;
        data->vel_x = ((int64_t)delta_fp * cfg->gain +
                       (int64_t)data->vel_x * cfg->blend) / 1000;
        data->vel_x = clamp_velocity(data->vel_x, cfg->limit_fp);
        data->total_movement += abs32(event->value);

        if (abs32(data->vel_x) > abs32(data->peak_vel_x)) {
            data->peak_vel_x = data->vel_x;
        }
    }

    /* ── Check if armed (flick detected) ── */
    bool vel_armed = false;
    if (cfg->axis != AXIS_X) vel_armed |= (abs32(data->peak_vel_y) >= cfg->start_fp);
    if (cfg->axis != AXIS_Y) vel_armed |= (abs32(data->peak_vel_x) >= cfg->start_fp);
    bool armed = vel_armed && data->total_movement >= cfg->move;

    /* ── Deceleration detection ── */
    if (armed) {
        bool decelerating = false;
        if (is_y && abs32(data->vel_y) < abs32(prev_vel_y)) {
            decelerating = true;
        }
        if (is_x && abs32(data->vel_x) < abs32(prev_vel_x)) {
            decelerating = true;
        }

        if (decelerating) {
            data->decel_count++;
            if (data->decel_count >= DECEL_CONFIRM_COUNT) {
                /* Confirmed deceleration — take over now */
                start_inertia(data, cfg);
                event->value = 0;
                return ZMK_INPUT_PROC_CONTINUE;
            }
        } else {
            data->decel_count = 0;
        }
    }

    /* Reset stop-detect fallback timer */
    k_work_reschedule(&data->stop_detect_work, K_MSEC(cfg->release_ms));

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ------------------------------------------------------------------ */
/* Device boilerplate                                                  */
/* ------------------------------------------------------------------ */

static int scroll_inertia_init(const struct device *dev) {
    struct scroll_inertia_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->stop_detect_work, stop_detect_handler);
    k_work_init_delayable(&data->inertia_tick_work, inertia_tick_handler);
    return 0;
}

static struct zmk_input_processor_driver_api scroll_inertia_driver_api = {
    .handle_event = scroll_inertia_handle_event,
};

#define SCROLL_INERTIA_INST(n)                                                \
    static struct scroll_inertia_data scroll_inertia_data_##n = {0};          \
    static const struct scroll_inertia_config scroll_inertia_config_##n = {   \
        .gain       = DT_INST_PROP(n, gain),                                  \
        .blend      = DT_INST_PROP(n, blend),                                 \
        .start_fp   = DT_INST_PROP(n, start) << FP_SHIFT,                    \
        .move       = DT_INST_PROP(n, move),                                  \
        .release_ms = DT_INST_PROP(n, release),                               \
        .fast_fp    = DT_INST_PROP(n, fast) << FP_SHIFT,                      \
        .decay_fast = DT_INST_PROP(n, decay_fast),                            \
        .decay_slow = DT_INST_PROP(n, decay_slow),                            \
        .stop_fp    = DT_INST_PROP(n, stop) << FP_SHIFT,                      \
        .scale      = DT_INST_PROP(n, scale),                                 \
        .scale_div  = DT_INST_PROP(n, scale_div),                             \
        .limit_fp   = DT_INST_PROP(n, limit) << FP_SHIFT,                    \
        .span_ms    = DT_INST_PROP(n, span),                                  \
        .tick_ms    = DT_INST_PROP(n, tick),                                   \
        .axis       = DT_INST_PROP(n, axis),                                   \
        .cancel     = DT_INST_PROP(n, cancel),                                 \
        .layer      = DT_INST_PROP(n, layer),                                  \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(n, scroll_inertia_init, NULL,                       \
                          &scroll_inertia_data_##n,                           \
                          &scroll_inertia_config_##n,                         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &scroll_inertia_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_INERTIA_INST)
