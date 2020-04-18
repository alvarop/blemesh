#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "os/os_cputime.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "hal/hal_timer.h"
#include "bsp/bsp.h"


#if MYNEWT_VAL(USE_BLE)
#include <host/ble_hs.h>
#endif

#define BLINK_TASK_PRI         (99)
#define BLINK_STACK_SIZE       (128)
struct os_task blink_task;
os_stack_t blink_task_stack[BLINK_STACK_SIZE];

static uint16_t timestamp = 0;

#if MYNEWT_VAL(USE_BLE)

#define BEACON_MAGIC 0x71A0

#define TYPE_ULTRASOUND 0x01
#define TYPE_PIR        0x02

typedef struct {
    uint16_t magic;
    uint32_t device_id;
    uint16_t type;
    uint16_t flags;
    uint16_t timestamp;
    uint32_t distance_mm;
} __attribute__((packed)) ble_beacon_t;

typedef struct {
    uint16_t magic;
    uint32_t device_id;
    uint16_t type;
    uint16_t flags;
    uint16_t timestamp;
    uint32_t active;
} __attribute__((packed)) ble_pir_beacon_t;

static ble_pir_beacon_t beacon_data;

static void ble_app_advertise();

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
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
static void ble_app_advertise() {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    hal_gpio_toggle(LED_3_PIN);

    beacon_data.magic = BEACON_MAGIC;
    beacon_data.device_id = NRF_FICR->DEVICEADDR[0];
    beacon_data.type = TYPE_PIR;
    beacon_data.timestamp = timestamp++;
    beacon_data.active = hal_gpio_read(PIR_PIN);
    beacon_data.flags = 0;

    fields = (struct ble_hs_adv_fields){ 0 };
    rc = ble_eddystone_set_adv_data_uid(&fields, &beacon_data, 0);
    assert(rc == 0);

    adv_params = (struct ble_gap_adv_params){ 0 };
    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_RANDOM,
        NULL,
        100,
        &adv_params,
        gap_event_cb,
        NULL);
    assert(rc == 0);

}

static void ble_app_on_sync(void) {
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
    ble_app_advertise();
}
#endif

void blink_task_fn(void *arg) {

    hal_gpio_init_out(LED_1_PIN, 0);
    hal_gpio_init_out(LED_2_PIN, 0);
    hal_gpio_init_out(LED_3_PIN, 0);

    hal_gpio_init_in(PIR_PIN, HAL_GPIO_PULL_DOWN);

    while(1) {
        hal_gpio_toggle(LED_1_PIN);
        hal_gpio_toggle(LED_2_PIN);
        os_time_delay(OS_TICKS_PER_SEC/2);
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
