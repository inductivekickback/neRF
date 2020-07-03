/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <power/power.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <nrfx_gpiote.h>
#include <settings/settings.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include <bluetooth/services/nerf.h>

#define DEVICE_NAME                 CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN             (sizeof(DEVICE_NAME) - 1)

#define DART_COUNT_MAX              24
#define DART_COUNT_UNKNOWN          0xFF
#define HAPTIC_DURATION_VALUE_MIN   16
#define HAPTIC_DURATION_VALUE_MAX   200 
#define BURST_DART_COUNT            3
#define REV_SPIN_UP_MS              1000
#define SWITCH_DEBOUNCE_MS          5

#define PLUNGER_IDLE_SPIN_MS        250
#define PLUNGER_PUSH_MS             125
#define PLUNGER_REBOUND_MS          30
#define PLUNGER_RESET_MS            25
#define PLUNGER_MARGIN_MS           100

#define LED_ERROR_BLINK_DELAY_MS    200

#define SYSTEM_OFF_DELAY_M          10
#define SYSTEM_OFF_ERR_DELAY_S      30

/* Convenience macros for working with different GPIO levels */
#define LED_ON_SET(pin)     (nrf_gpio_pin_set(pin))
#define LED_OFF_SET(pin)    (nrf_gpio_pin_clear(pin))
#define MOTOR_ON_SET(pin)   (nrf_gpio_pin_set(pin))
#define MOTOR_OFF_SET(pin)  (nrf_gpio_pin_clear(pin))
#define SW_ACTIVE_LOW(pin)  (!nrf_gpio_pin_read(pin))
#define SW_ACTIVE_HIGH(pin) (nrf_gpio_pin_read(pin))


typedef enum
{
    FIRING_MODE_SEMI,  /* One shot per plunger trigger press */
    FIRING_MODE_BURST, /* Three shots per plunger trigger press */
    FIRING_MODE_FULL,  /* Continuous shots while plunger trigger is pressed */
    FIRING_MODE_COUNT
} firing_mode_t;

static void rev_timer_expire(struct k_timer *timer_id);
static void debounce_timer_expire(struct k_timer *timer_id);
static void plunger_release(void);
static void sleep_timer_expire(struct k_timer *timer_id);
static void haptic_timer_expire(struct k_timer *timer_id);

/* These variables are only used when the blaster has a BT connection. */
static firing_mode_t   m_firing_mode                = FIRING_MODE_FULL;
static uint8_t         m_dart_count                 = DART_COUNT_UNKNOWN;
static bool            m_rev_trigger_overridden     = false;
static bool            m_plunger_trigger_overridden = false;
static bool            m_triggers_locked_out        = false;
static struct bt_conn *m_bt_conn                    = NULL;

static volatile bool m_revved = false;
static          bool m_firing = false;

static volatile bool m_rev_trig_sw      = false;
static volatile bool m_plunger_trig_sw  = false;
static volatile bool m_plunger_sw       = false;
static volatile bool m_sleep_mode_enter = false;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, NERF_UUID_SERVICE),
};

/* Use the system workqueue to send notifications from a timer ISR. */
static struct haptic_notification_ctx {
    struct k_work work;
    uint8_t       duration_remaining;
} m_haptic_notification_ctx;

K_MUTEX_DEFINE(m_dart_count_mutex);
K_SEM_DEFINE(m_dart_sem, 0, DART_COUNT_MAX);
K_SEM_DEFINE(m_plunger_sem, 0, 1);
K_TIMER_DEFINE(m_rev_timer, rev_timer_expire, NULL);
K_TIMER_DEFINE(m_rev_trig_debounce_timer, debounce_timer_expire, NULL);
K_TIMER_DEFINE(m_plunger_trig_debounce_timer, debounce_timer_expire, NULL);
K_TIMER_DEFINE(m_plunger_debounce_timer, debounce_timer_expire, NULL);
K_TIMER_DEFINE(m_sleep_timer, sleep_timer_expire, NULL);
K_TIMER_DEFINE(m_haptic_timer, haptic_timer_expire, NULL);


static void rev_trigger_override(bool override)
{
    m_rev_trigger_overridden = override;
    debounce_timer_expire(&m_rev_trig_debounce_timer);
    LOG_DBG("%d", override);
}


static void plunger_trigger_override(bool override)
{
    m_plunger_trigger_overridden = override;
    debounce_timer_expire(&m_plunger_trig_debounce_timer);
    LOG_DBG("%d", override);
}


static void bt_connected(struct bt_conn *conn, u8_t err)
{
    if (err) {
        LOG_ERR("bt_conn_cb.connected error (%u)", err);
        return;
    }

    LOG_DBG("Connected");

    k_timer_stop(&m_sleep_timer);

    m_bt_conn = bt_conn_ref(conn);
    LED_ON_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);
}


static void bt_disconnected(struct bt_conn *conn, u8_t reason)
{
    LOG_DBG("Disconnected (reason %u)", reason);

    k_timer_start(&m_sleep_timer, K_MINUTES(SYSTEM_OFF_DELAY_M), 0);

    if (m_bt_conn) {
        bt_conn_unref(m_bt_conn);
        m_bt_conn = NULL;
    }
    LED_OFF_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);

    /* Without a BT connection it doesn't make sense to track the dart count. */
    k_mutex_lock(&m_dart_count_mutex, K_FOREVER);
    m_dart_count = DART_COUNT_UNKNOWN;
    k_mutex_unlock(&m_dart_count_mutex);

    m_triggers_locked_out = false;

    if (m_rev_trigger_overridden) {
        rev_trigger_override(false);
    }

    if (m_plunger_trigger_overridden) {
        plunger_trigger_override(false);
    }
}


static void dart_count_set_cb(uint8_t darts_remaining)
{
    if ((DART_COUNT_MAX >= darts_remaining) || (DART_COUNT_UNKNOWN == darts_remaining)) {
        k_mutex_lock(&m_dart_count_mutex, K_FOREVER);
        m_dart_count = darts_remaining;
        k_mutex_unlock(&m_dart_count_mutex);
        LOG_DBG("%d", darts_remaining);
    } else {
        LOG_DBG("Invalid dart count: %d", darts_remaining);
    }
}


static uint8_t dart_count_get_cb(void)
{
    uint8_t darts_remaining;
    k_mutex_lock(&m_dart_count_mutex, K_FOREVER);
    darts_remaining = m_dart_count;
    k_mutex_unlock(&m_dart_count_mutex);
    return darts_remaining;
}


static void selective_fire_mode_set_cb(uint8_t mode)
{
    if (FIRING_MODE_COUNT > mode) {
        m_firing_mode = mode;
        LOG_DBG("%d", mode);
    } else {
        LOG_DBG("Invalid firing mode: %d", mode);
    }
}


static uint8_t selective_fire_mode_get_cb(void)
{
    return m_firing_mode;
}


static void rev_trigger_override_set_cb(bool override)
{
    if (override != m_rev_trigger_overridden) {
        rev_trigger_override(override);
    }
}


static bool rev_trigger_override_get_cb(void)
{
    return m_rev_trigger_overridden;
}


static void plunger_trigger_override_set_cb(bool override)
{
    if (override != m_plunger_trigger_overridden) {
        plunger_trigger_override(override);
    }
}


static bool plunger_trigger_override_get_cb(void)
{
    return m_plunger_trigger_overridden;
}


static void triggers_lockout_set_cb(bool lockout)
{
    if (m_triggers_locked_out != lockout) {
        m_triggers_locked_out = lockout;
        if (m_triggers_locked_out) {
            debounce_timer_expire(&m_plunger_trig_debounce_timer);
            debounce_timer_expire(&m_rev_trig_debounce_timer);
            LOG_DBG("Triggers locked out.");
        } else {
            LOG_DBG("Triggers no longer locked out.");
        }
    }
}


static bool triggers_lockout_get_cb(void)
{
    return m_triggers_locked_out;
}


static void haptic_feedback_set_cb(uint8_t duration_10ms)
{
    int duration;

    if ((HAPTIC_DURATION_VALUE_MIN > duration_10ms)||(HAPTIC_DURATION_VALUE_MAX < duration_10ms)) {
        LOG_DBG("Invalid duration: %d", duration_10ms);
        return;
    }

    duration = (10 * duration_10ms);

    k_timer_start(&m_haptic_timer, K_MSEC(duration), 0);
    MOTOR_ON_SET(DT_ALIAS_VIBE_EN_GPIOS_PIN);

    LOG_DBG("%dms", duration);
}


static struct bt_gatt_nerf_cb nerf_callbacks = {
    .dart_count_set_cb                  = dart_count_set_cb,
    .dart_count_get_cb                  = dart_count_get_cb,
    .selective_fire_mode_set_cb         = selective_fire_mode_set_cb,
    .selective_fire_mode_get_cb         = selective_fire_mode_get_cb,
    .rev_trigger_override_set_cb        = rev_trigger_override_set_cb,
    .rev_trigger_override_get_cb        = rev_trigger_override_get_cb,
    .plunger_trigger_override_set_cb    = plunger_trigger_override_set_cb,
    .plunger_trigger_override_get_cb    = plunger_trigger_override_get_cb,
    .triggers_lockout_set_cb            = triggers_lockout_set_cb,
    .triggers_lockout_get_cb            = triggers_lockout_get_cb,
    .haptic_feedback_set_cb             = haptic_feedback_set_cb,
};


static struct bt_conn_cb conn_callbacks = {
    .connected    = bt_connected,
    .disconnected = bt_disconnected,
};


/*
 * Turn off LEDs and motors if an error occurs or to prepare for sleep.
 */
static void outputs_disable(void)
{
    MOTOR_OFF_SET(DT_ALIAS_REV_EN_GPIOS_PIN);
    MOTOR_OFF_SET(DT_ALIAS_VIBE_EN_GPIOS_PIN);
    MOTOR_OFF_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);
    LED_OFF_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);
    LED_OFF_SET(DT_ALIAS_LED_G_EN_GPIOS_PIN);
    LED_OFF_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);
}


static void haptic_feedback_notification_send(struct k_work *item)
{
    struct haptic_notification_ctx *p_ctx;

    p_ctx = CONTAINER_OF(item, struct haptic_notification_ctx, work);
    if (m_bt_conn) {
        int err = bt_gatt_nerf_hf_notify(p_ctx->duration_remaining);
        if (err && (-EACCES != err)) {
            LOG_ERR("bt_gatt_nerf_hf_notify error (%d)", err);
        }
    }
}


static void haptic_timer_expire(struct k_timer *timer_id)
{
    MOTOR_OFF_SET(DT_ALIAS_VIBE_EN_GPIOS_PIN);

    /* Use the system workqueue to call into the BT stack. */
    m_haptic_notification_ctx.duration_remaining = 0;
    k_work_submit(&m_haptic_notification_ctx.work);
}


/*
 * Expire m_sleep_timer if no triggers are pressed for SYSTEM_OFF_DELAY_M minutes.
 */
static void sleep_timer_expire(struct k_timer *timer_id)
{
    /* Set the flag to enable sleep mode and release the main thread. */
    m_sleep_mode_enter = true;
    k_sem_give(&m_dart_sem);
}


/**
 * Put the nRF52840 in its deepest sleep state.
 *
 * NOTE: sys_pm_force_power_state can't be called from an ISR so this function
 *       is called from main.
 */
static void system_off_enter(void)
{
    outputs_disable();
    nrf_gpio_cfg_sense_set(DT_ALIAS_REV_TRIG_SW_GPIOS_PIN, NRF_GPIO_PIN_SENSE_LOW);
    sys_pm_force_power_state(SYS_POWER_STATE_DEEP_SLEEP_1);
    k_sleep(K_MSEC(1));

    LOG_ERR("System off failed");
    while (1) {
        LED_ON_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);
        k_sleep(LED_ERROR_BLINK_DELAY_MS);
        LED_OFF_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);
        k_sleep(LED_ERROR_BLINK_DELAY_MS);
    }
}


/*
 * Sample switch levels after they have had time to settle.
 */
static void debounce_timer_expire(struct k_timer *timer_id)
{
    bool new_state;

    if (timer_id == &m_plunger_debounce_timer) {
        new_state = SW_ACTIVE_LOW(DT_ALIAS_PLUNGER_SW_GPIOS_PIN);
        if (new_state == m_plunger_sw) {
            return;
        }
        m_plunger_sw = new_state;
        k_sem_give(&m_plunger_sem);
    } else if (timer_id == &m_rev_trig_debounce_timer) {
        if (!m_triggers_locked_out && !m_rev_trigger_overridden) {
            new_state = SW_ACTIVE_LOW(DT_ALIAS_REV_TRIG_SW_GPIOS_PIN);
        } else {
            new_state = m_rev_trigger_overridden;
        }
        if (new_state == m_rev_trig_sw) {
            return;
        }
        m_rev_trig_sw = new_state;
        if (m_rev_trig_sw) {
            if (!m_firing) {
                MOTOR_ON_SET(DT_ALIAS_REV_EN_GPIOS_PIN);
            }
            k_timer_start(&m_rev_timer, K_MSEC(REV_SPIN_UP_MS), 0);
            LOG_DBG("Revving...");
        } else {
            m_revved = false;
            if (!m_firing) {
                MOTOR_OFF_SET(DT_ALIAS_REV_EN_GPIOS_PIN);
                LED_OFF_SET(DT_ALIAS_LED_G_EN_GPIOS_PIN);
            }
            k_timer_stop(&m_rev_timer);
            LOG_DBG("Rev trigger released");
        }
    } else if (timer_id == &m_plunger_trig_debounce_timer) {
        if (!m_triggers_locked_out && !m_plunger_trigger_overridden) {
            new_state = SW_ACTIVE_LOW(DT_ALIAS_PLUNGER_TRIG_SW_GPIOS_PIN);
        } else {
            new_state = m_plunger_trigger_overridden;
        }
        if (new_state == m_plunger_trig_sw) {
            return;
        }
        m_plunger_trig_sw = new_state;
        if (m_plunger_trig_sw) {
            LOG_DBG("Plunger trigger pressed");
            if (m_revved) {
                plunger_release();
            }
        } else {
            LOG_DBG("Plunger trigger released");
            if (FIRING_MODE_FULL == m_firing_mode) {
                k_sem_reset(&m_dart_sem);
            }
        }
    }
}


/*
 * Allow the wheels that propel the dart to spin up to speed before allowing darts to be fired.
 */
static void rev_timer_expire(struct k_timer *timer_id)
{
    LOG_DBG("Ready to fire");
    m_revved = true;
    LED_ON_SET(DT_ALIAS_LED_G_EN_GPIOS_PIN);

    if (m_plunger_trig_sw) {
        plunger_release();
    }
}


/*
 * Begin debouncing important switches and reset the sleep timer (when there is no BT connection).
 */
static void switch_handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (NULL == m_bt_conn) {
        k_timer_start(&m_sleep_timer, K_MINUTES(SYSTEM_OFF_DELAY_M), 0);
    }

    switch (pin) {
        case DT_ALIAS_REV_TRIG_SW_GPIOS_PIN:
            if (!m_triggers_locked_out && !m_rev_trigger_overridden) {
                k_timer_start(&m_rev_trig_debounce_timer, K_MSEC(SWITCH_DEBOUNCE_MS), 0);
            }
            break;
        case DT_ALIAS_PLUNGER_TRIG_SW_GPIOS_PIN:
            if (!m_triggers_locked_out && !m_plunger_trigger_overridden) {
                k_timer_start(&m_plunger_trig_debounce_timer, K_MSEC(SWITCH_DEBOUNCE_MS), 0);
            }
            break;
        case DT_ALIAS_PLUNGER_SW_GPIOS_PIN:
            k_timer_start(&m_plunger_debounce_timer, K_MSEC(SWITCH_DEBOUNCE_MS), 0);
            break;
        default:
            break;
    }
}


/*
 * Give the semaphore once for every dart that is ready to be fired. This semaphore releases
 * the main thread so it can begin firing.
 */
static void plunger_release(void)
{
    int count=0;

    if (m_firing) {
        return;
    }

    k_sem_reset(&m_dart_sem);

    switch (m_firing_mode) {
    case FIRING_MODE_SEMI:
        count = MIN(1, m_dart_count);
        break;
    case FIRING_MODE_BURST:
        count = MIN(BURST_DART_COUNT, m_dart_count);
        break;
    case FIRING_MODE_FULL:
        count = MIN(DART_COUNT_MAX, m_dart_count);
        break;
    default:
        break;
    }

    for (int i=0; i < count; i++) {
        k_sem_give(&m_dart_sem);
    }
}


/*
 * Initialize the switches and enable an interrupt handler to process them.
 */
static int switches_init(void)
{
    nrfx_err_t err;

    IRQ_CONNECT(DT_NORDIC_NRF_GPIOTE_GPIOTE_0_IRQ_0,
                    DT_NORDIC_NRF_GPIOTE_GPIOTE_0_IRQ_0_PRIORITY,
                    nrfx_isr,
                    nrfx_gpiote_irq_handler,
                    0);

    /* NOTE: The interrupt priority passed as the parameter here is ignored,
     *       per nrfx_glue.h, because it is set via IRQ_CONNECT.
     */
    err = nrfx_gpiote_init(0);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    nrfx_gpiote_in_config_t const in_config = {
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,
        .pull = NRF_GPIO_PIN_PULLUP,
        .is_watcher = false,
        .hi_accuracy = false,
        .skip_gpio_setup = false,
    };

    err = nrfx_gpiote_in_init(DT_ALIAS_PLUNGER_TRIG_SW_GPIOS_PIN, &in_config, switch_handle);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    err = nrfx_gpiote_in_init(DT_ALIAS_REV_TRIG_SW_GPIOS_PIN, &in_config, switch_handle);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    err = nrfx_gpiote_in_init(DT_ALIAS_PLUNGER_SW_GPIOS_PIN, &in_config, switch_handle);
    if (NRFX_SUCCESS != err) {
        return err;
    }

#if CONFIG_NERF_SAFETY_FEATURES_ENABLED
    nrf_gpio_cfg_input(DT_ALIAS_DRUM_DETECT_SW_GPIOS_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DT_ALIAS_JAM_DOOR_SW_GPIOS_PIN, NRF_GPIO_PIN_PULLUP);
#endif

    /* Note the original position for use later with debouncing. */
    m_rev_trig_sw = SW_ACTIVE_LOW(DT_ALIAS_REV_TRIG_SW_GPIOS_PIN);
    m_plunger_trig_sw = SW_ACTIVE_LOW(DT_ALIAS_PLUNGER_TRIG_SW_GPIOS_PIN);
    m_plunger_sw = SW_ACTIVE_LOW(DT_ALIAS_PLUNGER_SW_GPIOS_PIN);

    nrfx_gpiote_in_event_enable(DT_ALIAS_PLUNGER_TRIG_SW_GPIOS_PIN, true);
    nrfx_gpiote_in_event_enable(DT_ALIAS_REV_TRIG_SW_GPIOS_PIN, true);
    nrfx_gpiote_in_event_enable(DT_ALIAS_PLUNGER_SW_GPIOS_PIN, true);

    return NRFX_SUCCESS;
}


/*
 * Enable the LED and motor outputs and set them to off.
 */
static void outputs_init(void)
{
    LED_OFF_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_LED_R_EN_GPIOS_PIN);
 
    LED_OFF_SET(DT_ALIAS_LED_G_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_LED_G_EN_GPIOS_PIN);

    LED_OFF_SET(DT_ALIAS_LED_B_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_LED_B_EN_GPIOS_PIN);

    MOTOR_OFF_SET(DT_ALIAS_REV_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_REV_EN_GPIOS_PIN);

    MOTOR_OFF_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);

    MOTOR_OFF_SET(DT_ALIAS_VIBE_EN_GPIOS_PIN);
    nrf_gpio_cfg_output(DT_ALIAS_VIBE_EN_GPIOS_PIN);
}


/*
 * Initialize the system and manage the dart firing mechanism.
 */
void main(void)
{
    int err;

    k_work_init(&m_haptic_notification_ctx.work, haptic_feedback_notification_send);

    /* Prevent deep sleep (SYSTEMOFF) from being entered on long timeouts. */
    sys_pm_ctrl_disable_state(SYS_POWER_STATE_DEEP_SLEEP_1);
    k_timer_start(&m_sleep_timer, K_MINUTES(SYSTEM_OFF_DELAY_M), 0);

    outputs_init();

    err = switches_init();
    if (NRFX_SUCCESS != err) {
        LOG_ERR("switches_init error (%d)", err);
        goto err_exit;
    }

    bt_conn_cb_register(&conn_callbacks);
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable error (%d)", err);
        goto err_exit;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_gatt_nerf_init(&nerf_callbacks);
    if (err) {
        LOG_ERR("bt_gatt_nerf_init error (%d)", err);
        goto err_exit;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("bt_le_adv_start error (%d)", err);
        goto err_exit;
    }

    LOG_DBG("Mastodon initialization complete");

    while (1) {
        k_sem_take(&m_dart_sem, K_FOREVER);

        if (m_sleep_mode_enter) {
            system_off_enter();
        }

#if CONFIG_NERF_SAFETY_FEATURES_ENABLED
        if (SW_ACTIVE_LOW(DT_ALIAS_JAM_DOOR_SW_GPIOS_PIN)) {
            LOG_ERR("Jam door fault");
            goto err_exit;
        }
    
        if (!SW_ACTIVE_LOW(DT_ALIAS_DRUM_DETECT_SW_GPIOS_PIN)) {
            LOG_ERR("Drum detect fault");
            goto err_exit;
        }
#endif
        m_firing = true;
        LED_ON_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);

        /* m_plunger_sw should be false when at rest. It will go true when the gear that
         * drives it moves through its idle section and begins to re-engage it. */
        k_sem_reset(&m_plunger_sem);
        MOTOR_ON_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);
        if (0 != k_sem_take(&m_plunger_sem,
                                K_MSEC(PLUNGER_IDLE_SPIN_MS + PLUNGER_MARGIN_MS))) {
            goto err_exit;
        }

        /* Let the plunger run until m_plunger_sw goes false. */
        if (0 != k_sem_take(&m_plunger_sem,
                                K_MSEC(PLUNGER_PUSH_MS + PLUNGER_MARGIN_MS))) {
            goto err_exit;
        }
        MOTOR_OFF_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);

        /* Allow plunger spring to retract the plunger. */
        if (0 != k_sem_take(&m_plunger_sem,
                                K_MSEC(PLUNGER_REBOUND_MS + PLUNGER_MARGIN_MS))) {
            goto err_exit;
        }

        MOTOR_ON_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);
        if (0 != k_sem_take(&m_plunger_sem,
                                K_MSEC(PLUNGER_RESET_MS + PLUNGER_MARGIN_MS))) {
            goto err_exit;
        }
        MOTOR_OFF_SET(DT_ALIAS_PLUNGER_EN_GPIOS_PIN);

        if (!m_rev_trig_sw) {
            MOTOR_OFF_SET(DT_ALIAS_REV_EN_GPIOS_PIN);
            LED_OFF_SET(DT_ALIAS_LED_G_EN_GPIOS_PIN);
        }

        LED_OFF_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);
        m_firing = false;

        k_mutex_lock(&m_dart_count_mutex, K_FOREVER);
        if (DART_COUNT_UNKNOWN != m_dart_count) {
            if (0 == m_dart_count) {
                m_dart_count = DART_COUNT_UNKNOWN;
            } else {
                m_dart_count--;
            }
        }

        if (m_bt_conn) {
            err = bt_gatt_nerf_dc_notify(m_dart_count);
            if (err && (-EACCES != err)) {
                LOG_ERR("bt_gatt_nerf_dc_notify error (%d)", err);
                k_mutex_unlock(&m_dart_count_mutex);
                goto err_exit;
            }
        }

        if (0 == m_dart_count) {
            if (m_rev_trigger_overridden) {
                rev_trigger_override(false);
                if (m_bt_conn) {
                    err = bt_gatt_nerf_rto_notify(m_rev_trigger_overridden);
                    if (err && (-EACCES != err)) {
                        LOG_ERR("bt_gatt_nerf_rto_notify error (%d)", err);
                        k_mutex_unlock(&m_dart_count_mutex);
                        goto err_exit;
                    }
                }
            }
            if (m_plunger_trigger_overridden) {
                plunger_trigger_override(false);
                if (m_bt_conn) {
                    err = bt_gatt_nerf_pto_notify(m_plunger_trigger_overridden);
                    if (err && (-EACCES != err)) {
                        LOG_ERR("bt_gatt_nerf_pto_notify error (%d)", err);
                        k_mutex_unlock(&m_dart_count_mutex);
                        goto err_exit;
                    }
                }
            }
        } else if (0 == k_sem_count_get(&m_dart_sem)) {
            if (m_plunger_trigger_overridden) {
                plunger_trigger_override(false);
                if (m_bt_conn) {
                    err = bt_gatt_nerf_pto_notify(m_plunger_trigger_overridden);
                    if (err && (-EACCES != err)) {
                        LOG_ERR("bt_gatt_nerf_pto_notify error (%d)", err);
                        k_mutex_unlock(&m_dart_count_mutex);
                        goto err_exit;
                    }
                }
            }
        }

        k_mutex_unlock(&m_dart_count_mutex);
        LOG_DBG("Fired one dart");
    }

err_exit:
    /* Put the system into a safe state. */
    NRFX_IRQ_DISABLE(nrfx_get_irq_number(NRF_GPIOTE));

    if (m_bt_conn) {
        err = bt_conn_disconnect(m_bt_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (err) {
            LOG_ERR("bt_conn_disconnect error (%d)", err);
        }
    } else {
        err = bt_le_adv_stop();
        if (err) {
            LOG_ERR("bt_le_adv_stop error (%d)", err);
        }
    }

    outputs_disable();

    k_timer_start(&m_sleep_timer, K_SECONDS(SYSTEM_OFF_ERR_DELAY_S), 0);

    LOG_ERR("err_exit shutdown");

    while (1) {
        LED_ON_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);
        k_sleep(LED_ERROR_BLINK_DELAY_MS);
        LED_OFF_SET(DT_ALIAS_LED_R_EN_GPIOS_PIN);
        k_sleep(LED_ERROR_BLINK_DELAY_MS);

        if (m_sleep_mode_enter) {
            system_off_enter();
        }
    }
}
