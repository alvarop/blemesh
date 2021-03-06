#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "os/os_cputime.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "hal/hal_timer.h"
#include "bsp/bsp.h"
#include "ws2812.h"


#if MYNEWT_VAL(USE_BLE)
#include <host/ble_hs.h>
#endif

#define BLINK_TASK_PRI         (99)
#define BLINK_STACK_SIZE       (128)
struct os_task blink_task;
os_stack_t blink_task_stack[BLINK_STACK_SIZE];

#define ULTRASOUND_TASK_PRI         (50)
#define ULTRASOUND_STACK_SIZE       (128)
struct os_task ultrasound_task;
os_stack_t ultrasound_task_stack[ULTRASOUND_STACK_SIZE];


#if MYNEWT_VAL(USE_BLE)

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

    fields = (struct ble_hs_adv_fields){ 0 };
    rc = ble_eddystone_set_adv_data_url(
        &fields,
        BLE_EDDYSTONE_URL_SCHEME_HTTPS,
        "alvarop.com",
        11,
        BLE_EDDYSTONE_URL_SUFFIX_NONE,
        0);
    assert(rc == 0);

    adv_params = (struct ble_gap_adv_params){ 0 };
    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_RANDOM,
        NULL,
        1000,
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


// Timeout (in microseconds) before giving up
#define TIMEOUT_US 400000

struct hal_timer ultrasound_timeout_timer;

static struct os_sem ultrasound_processing_sem;

static volatile uint32_t start_time;
static volatile uint32_t stop_time;

// Measure pulse width here
static void ultrasound_irq(void *arg) {
    os_error_t err;

    if (hal_gpio_read(ECHO_PIN) == 1) {
        start_time = hal_timer_read(1);
    } else {
        stop_time = hal_timer_read(1);
        err = os_sem_release(&ultrasound_processing_sem);
        assert(err == OS_OK);

        // Re-set ir timeout
        hal_timer_stop(&ultrasound_timeout_timer);
    }
}

static void ultrasound_timeout_cb(void *arg) {
    os_error_t err;

    // No echo detected :(
    err = os_sem_release(&ultrasound_processing_sem);
    assert(err == OS_OK);

    console_printf("timeout!\n");

    start_time = 0;
    stop_time = 0;
}


static uint32_t last_distance = 0;

#define MAX_MM 300

#define MAX_BRIGHTNESS 32

void update_leds(uint32_t distance_mm) {

    for(uint16_t led=1; led < WS2812_NUM_PIXELS; led++) {
        uint32_t threshold = MAX_MM * (WS2812_NUM_PIXELS-led+1) /(WS2812_NUM_PIXELS-1);
        int32_t brightness = threshold - distance_mm;
        if(brightness > 255) {
            brightness = 255;
        } else if (brightness < 0) {
            brightness = 0;
        }

        brightness = (brightness * MAX_BRIGHTNESS)/ 256;

        // console_printf("%d %ld %ld\n", led, distance_mm, threshold);
        if(distance_mm < (threshold)) {
            ws2812_set_pixel(led, brightness, brightness, brightness);
        } else {
            ws2812_set_pixel(led, 0, 0, 0);
        }
    }

    ws2812_write();
}

void blink_task_fn(void *arg) {

    hal_gpio_init_out(LED_1_PIN, 0);
    hal_gpio_init_out(LED_2_PIN, 0);
    hal_gpio_init_out(LED_3_PIN, 0);

    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/64);
        update_leds(last_distance);
    }
}

#define UM_PER_US 343

void ultrasound_task_fn(void *arg) {

    os_error_t err;

    ws2812_init();

    err = os_sem_init(&ultrasound_processing_sem, 1);
    assert(err == OS_OK);

    hal_gpio_irq_init(ECHO_PIN, ultrasound_irq, NULL,
        HAL_GPIO_TRIG_BOTH, HAL_GPIO_PULL_NONE);

    // 1MHz timer to measure incoming pulse edge times
    hal_timer_config(1, 1000000);

    // Timeout used to process a packet after some time without edges
    hal_timer_set_cb(1, &ultrasound_timeout_timer, ultrasound_timeout_cb, NULL);

    hal_gpio_irq_enable(ECHO_PIN);

    hal_gpio_init_out(TRIG_PIN, 0);

    start_time = 0;
    stop_time = 0;

    while (1) {

        os_time_delay(OS_TICKS_PER_SEC/16);

        hal_gpio_write(LED_3_PIN, 1);
        hal_gpio_write(LED_2_PIN, 1);
        hal_gpio_write(TRIG_PIN, 1);
        os_cputime_delay_usecs(10);
        hal_gpio_write(TRIG_PIN, 0);
        hal_gpio_write(LED_3_PIN, 0);

        hal_timer_start(&ultrasound_timeout_timer, TIMEOUT_US);

        err = os_sem_pend(&ultrasound_processing_sem, 0xFFFFFFFF);
        err = os_sem_pend(&ultrasound_processing_sem, 0xFFFFFFFF);
        os_sem_release(&ultrasound_processing_sem);

        hal_gpio_write(LED_2_PIN, 0);

        if(start_time && stop_time) {
            uint32_t diff_us = stop_time - start_time;
            uint32_t diff_um = (diff_us * 343)/2;
            uint32_t distance_mm = diff_um/1000;
            // console_printf("%ldmm\n", distance_mm);
            if(distance_mm < (MAX_MM + 25)) {
                last_distance = distance_mm;
            } else {
                if(last_distance <= (MAX_MM + 25)){
                    last_distance += 25;
                }
            }
        }

        start_time = 0;
        stop_time = 0;
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

    os_task_init(
        &ultrasound_task,
        "ultrasound_task",
        ultrasound_task_fn,
        NULL,
        ULTRASOUND_TASK_PRI,
        OS_WAIT_FOREVER,
        ultrasound_task_stack,
        ULTRASOUND_STACK_SIZE);

#if MYNEWT_VAL(USE_BLE)
     ble_hs_cfg.sync_cb = ble_app_on_sync;
#endif

    while(1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    assert(0);

    return 0;
}
