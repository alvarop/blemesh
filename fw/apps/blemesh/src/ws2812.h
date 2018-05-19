#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int ws2812_init(void);

void ws2812_write(void);

void ws2812_set_pixel(uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue);

#define WS2812_NUM_PIXELS 9

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */
