#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "console/console.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"

#if MYNEWT_VAL(USE_BLE)
#include <host/ble_hs.h>
#endif

#define BLINK_TASK_PRI         (99)
#define BLINK_STACK_SIZE       (64)
struct os_task blink_task;
os_stack_t blink_task_stack[BLINK_STACK_SIZE];

#if MYNEWT_VAL(USE_BLE)
#define BEACON_MAGIC 0x0578

typedef struct {
    uint16_t magic;
    uint32_t device_id;
    uint16_t timestamp;
    uint8_t sensor_ch;
    uint8_t fridge_state;
    int16_t temperature;
    int16_t humidity;
    uint16_t flags;
} __attribute__((packed)) ble_beacon_t;

static ble_beacon_t beacon_data;

static void ble_app_advertise(bool);

static uint8_t adv_sensor_ch;

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise(false);
        break;
    }
    return 0;
}

static void ble_app_set_addr(void) {
    ble_addr_t addr;
    int rc;

    rc = ble_hs_id_gen_rnd(1, &addr);
    assert(rc == 0);

    rc = ble_hs_id_set_rnd(addr.val);
    assert(rc == 0);
}

// Use a BLE_CH_ADV_TIME_MS long beacon to send each individual
// channel's data
static void ble_app_advertise(bool first_tx) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    if(first_tx) {
        adv_sensor_ch = 0;
        beacon_data.device_id = NRF_FICR->DEVICEADDR[0];
        beacon_data.timestamp = timestamp;
        beacon_data.fridge_state = fridge_running;
        beacon_data.flags = 0;
    }

    // Find the next enabled sensor
    while(adv_sensor_ch < MAX_SENSORS && !sensors[adv_sensor_ch].enabled) {
        adv_sensor_ch++;
    }

    if(adv_sensor_ch < MAX_SENSORS) {
        beacon_data.sensor_ch = adv_sensor_ch;
        beacon_data.temperature = sensors[adv_sensor_ch].temperature;
        beacon_data.humidity = sensors[adv_sensor_ch].humidity;

        fields = (struct ble_hs_adv_fields){ 0 };
        rc = ble_eddystone_set_adv_data_uid(&fields, &beacon_data);
        assert(rc == 0);

        adv_params = (struct ble_gap_adv_params){ 0 };
        rc = ble_gap_adv_start(
            BLE_OWN_ADDR_RANDOM,
            NULL,
            MYNEWT_VAL(BLE_CH_ADV_TIME_MS),
            &adv_params,
            gap_event_cb,
            NULL);
        assert(rc == 0);

        adv_sensor_ch++;
    }
}

static void ble_app_on_sync(void) {
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
}
#endif


void blink_task_fn(void *arg) {

    hal_gpio_init_out(LED_1_PIN, 0);
    hal_gpio_init_out(LED_2_PIN, 0);
    hal_gpio_init_out(LED_3_PIN, 0);

    uint8_t counter = 0;

    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/2);

        hal_gpio_write(LED_1_PIN, !!(counter & 1));
        hal_gpio_write(LED_2_PIN, !!(counter & 2));
        hal_gpio_write(LED_3_PIN, !!(counter & 4));

        os_time_delay(1);

        hal_gpio_write(LED_1_PIN, 0);
        hal_gpio_write(LED_2_PIN, 0);
        hal_gpio_write(LED_3_PIN, 0);
        counter++;
    }

}

int
main(int argc, char **argv)
{
    sysinit();

    os_task_init(
        &blink_task,
        "blink_task",
        blink_task_fn,
        NULL,
        BLINK_TASK_PRI,
        OS_WAIT_FOREVER,
        blink_task_stack,
        BLINK_STACK_SIZE);

#if MYNEWT_VAL(USE_BLE)
     ble_hs_cfg.sync_cb = ble_app_on_sync;
#endif

    while(1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    assert(0);

    return 0;
}
