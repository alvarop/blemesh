#include <assert.h>
#include <os/os.h>

/* ADC */
#include "myadc/myadc.h"
#include <console/console.h>
#include <adc/adc.h>
#include <adc_nrf52/adc_nrf52.h>
#include <nrfx.h>
#include <nrfx_config.h>
#include <nrf_saadc.h>

#define ADC_NUMBER_SAMPLES (2)
#define ADC_NUMBER_CHANNELS (1)

nrfx_saadc_config_t adc_config = NRFX_SAADC_DEFAULT_CONFIG;

void *
adc_init(void)
{
    struct adc_dev *adc;

    adc = (struct adc_dev *) os_dev_open("adc0", 0, &adc_config);
    assert(adc != NULL);

    return adc;
}

int adc_init_ch(struct adc_dev *adc, uint8_t ch, nrf_saadc_input_t saadc_in) {
    if (adc == NULL) {
        return -1;
    } else {

        // Channel config pin is ch + 1
        nrf_saadc_channel_config_t cc = \
            NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(saadc_in);
        cc.gain = NRF_SAADC_GAIN1_6;
        cc.reference = NRF_SAADC_REFERENCE_INTERNAL;

        return adc_chan_config(adc, ch, &cc);
    }
}
