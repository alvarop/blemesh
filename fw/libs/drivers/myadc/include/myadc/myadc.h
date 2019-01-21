#ifndef _NRF52_ADC_H_
#define _NRF52_ADC_H_

#include <stdint.h>
#include <adc/adc.h>
#include <nrfx_saadc.h>

void * adc_init(void);
int adc_init_ch(struct adc_dev *adc, uint8_t ch, nrf_saadc_input_t saadc_in);

#endif /* _NRF52_ADC_H_ */
