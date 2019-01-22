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

#include <adc/adc.h>
#include <myadc/myadc.h>

#if MYNEWT_VAL(USE_BLE)
#include <host/ble_hs.h>
#endif

#define LED_TASK_PRI         (99)
#define LED_STACK_SIZE       (128)
struct os_task led_task;
os_stack_t led_task_stack[LED_STACK_SIZE];

#define WATER_TASK_PRI         (98)
#define WATER_STACK_SIZE       (128)
struct os_task water_task;
os_stack_t water_task_stack[WATER_STACK_SIZE];

static uint32_t distance_mm = 2000;
// static uint16_t timestamp = 0;

static uint16_t pixel_brightness;

typedef enum {
    LED_IDLE = 0,
    LED_FADING_IN,
    LED_FADING_OUT,
} led_state_t;

static led_state_t led_state = LED_IDLE;

#if MYNEWT_VAL(USE_BLE)

static int ble_app_gap_event(struct ble_gap_event *event, void *arg);

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

// static ble_beacon_t beacon_data;

static void ble_app_scan(void) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    // Don't discard any packets!
    disc_params.filter_duplicates = 0;

    // Don't send follow-up requests
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_app_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}


#define TYPE_ULTRASOUND 0x01
#define TYPE_PIR        0x02

static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {

    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        if (fields.uuids16 != NULL && fields.uuids16->value == 0xFEAA) {
            ble_beacon_t *data = (ble_beacon_t*) &fields.svc_data_uuid16[4];
            if(data->magic == BEACON_MAGIC) {
                if(data->type == TYPE_ULTRASOUND) {
                    distance_mm = data->distance_mm;
                    hal_gpio_write(LED_2_PIN, 1);
                    // update_leds(distance_mm);
                    hal_gpio_write(LED_2_PIN, 0);
                    // printf("rx %ld\n", data->distance_mm);
                } else if(data->type == TYPE_PIR) {
                    hal_gpio_write(LED_3_PIN, data->distance_mm);
                    if(data->distance_mm == 1) {
                        led_state = LED_FADING_IN;
                    } else {
                        led_state = LED_FADING_OUT;
                    }
                }
            }
        }

        return 0;

    default:
        return 0;
    }
}

static void ble_app_set_addr(void) {
    ble_addr_t addr;
    int rc;

    rc = ble_hs_id_gen_rnd(1, &addr);
    assert(rc == 0);

    rc = ble_hs_id_set_rnd(addr.val);
    assert(rc == 0);
}

static void ble_app_on_sync(void) {

    ble_app_set_addr();

    ble_app_scan();
}
#endif


#define MAX_MM 1500

#define MAX_BRIGHTNESS 64


void update_leds() {

    for(uint16_t led=1; led < WS2812_NUM_PIXELS; led++) {

        int32_t brightness = pixel_brightness;
        if(brightness > MAX_BRIGHTNESS) {
            brightness = MAX_BRIGHTNESS;
        }

        ws2812_set_pixel(led, brightness, brightness, brightness);
    }

    ws2812_write();

    switch(led_state) {
        case LED_FADING_IN: {
            if(pixel_brightness < 512) {
                pixel_brightness += 4;
            } else {
                led_state = LED_IDLE;
            }

            break;
        }
        case LED_FADING_OUT: {
            if(pixel_brightness > 0) {
                pixel_brightness--;
            } else {
                led_state = LED_IDLE;
            }

            break;
        }
        case LED_IDLE: {
            break;
        }
    }
}

#define MOISTURE (4)
#define MOISTURE_AIN (2)
#define MOISTURE_SAADC NRF_SAADC_INPUT_AIN2

static struct adc_dev *adc;

int16_t moisture_read() {
    int light_level = 0;
    adc_chan_read(adc, MOISTURE_AIN, &light_level);

    return (uint16_t)adc_result_mv(adc, MOISTURE_AIN, light_level);
}

void moisture_init() {
    adc = adc_init();

    adc_init_ch(adc, MOISTURE_AIN, MOISTURE_SAADC);
}


void led_task_fn(void *arg) {

    ws2812_init();

    hal_gpio_init_out(LED_1_PIN, 0);
    hal_gpio_init_out(LED_2_PIN, 0);
    hal_gpio_init_out(LED_3_PIN, 0);


    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/64);

        update_leds();
        // hal_gpio_write(LED_1_PIN, 1);
        // os_time_delay(1);
        // hal_gpio_write(LED_1_PIN, 0);

    }
}

static uint32_t watering = 0;

void water_task_fn(void *arg) {
    uint8_t print_count = 0;
    moisture_init();

    hal_gpio_init_out(TRIG_PIN, 0);

    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/32);

        if (distance_mm < 1000 || distance_mm > 2500) {
            watering = 32 * 5; // 5 second hold and max time
        } else {
            if (watering > 0) {
                watering--;
            }
        }

        int16_t moisture = moisture_read();
        if ((moisture < 800) && (watering > 0)) {
            hal_gpio_write(TRIG_PIN, 1);
        } else {
            hal_gpio_write(TRIG_PIN, 0);
        }

        if((print_count++ & 0x1F) == 0){
            printf("moisture: %d\ndistance: %ld\n", moisture, distance_mm);
        }
    }
}

int
main(int argc, char **argv)
{
    sysinit();

    os_task_init(
        &led_task,
        "led_task",
        led_task_fn,
        NULL,
        LED_TASK_PRI,
        OS_WAIT_FOREVER,
        led_task_stack,
        LED_STACK_SIZE);

    os_task_init(
        &water_task,
        "water_task",
        water_task_fn,
        NULL,
        WATER_TASK_PRI,
        OS_WAIT_FOREVER,
        water_task_stack,
        WATER_STACK_SIZE);

#if MYNEWT_VAL(USE_BLE)
     ble_hs_cfg.sync_cb = ble_app_on_sync;
#endif

    while(1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    assert(0);

    return 0;
}
