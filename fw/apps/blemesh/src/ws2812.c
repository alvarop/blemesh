#include <stdio.h>
#include <stdlib.h>
#include "bsp/bsp.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "nrf.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "nrfx.h"
#include "nrfx_spim.h"
#include "ws2812.h"

static nrfx_spim_t spi_dev = NRFX_SPIM_INSTANCE(0);

// Number of actual bytes it takes to encode WS2812 byte
#define BYTE_LEN (9)

// First and last bytes must be zero
static uint8_t pattern[BYTE_LEN * 3 * WS2812_NUM_PIXELS + 2];


// Look up table for all 256 bytes
static const uint8_t ws2812_lut[256][BYTE_LEN] = {
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xE0, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x70, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x38, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1C, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0E, 0x07, 0xC3, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0x03, 0xE1, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0x81, 0xF0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xC0, },
  {0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x87, 0xC3, 0xE1, 0xF0, },
};

void ws2812_set_pixel(uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue) {
    if (pixel < WS2812_NUM_PIXELS) {
        uint8_t *pixel_addr = &pattern[1 + pixel * BYTE_LEN * 3];
        memcpy(&pixel_addr[0], ws2812_lut[green], BYTE_LEN);
        memcpy(&pixel_addr[BYTE_LEN], ws2812_lut[red], BYTE_LEN);
        memcpy(&pixel_addr[BYTE_LEN * 2], ws2812_lut[blue], BYTE_LEN);
    }
}

void ws2812_write() {
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TX(&pattern, sizeof(pattern));
    nrfx_spim_xfer(&spi_dev, &xfer, 0);
}

int ws2812_init(void){
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency = NRF_SPIM_FREQ_8M;

    spi_config.mosi_pin = 6;

    // Unused pins
    spi_config.sck_pin = 29;
    spi_config.miso_pin = NRFX_SPIM_PIN_NOT_USED;

    // Workaround for NRFX_SPIM library bug.
    // If NRFX_SPIM_PIN_NOT_USED is used, the library takes it as 'pin 0'
    // and continues using it as a SS pin
    // Pin 2 is not connected on the 32-pin package
    spi_config.ss_pin = 2;

    uint32_t rval = nrfx_spim_init(&spi_dev, &spi_config, NULL, NULL);
    if (rval != NRFX_SUCCESS) {
        console_printf("spim_init error %08lX\n", rval);
    } else {
        console_printf("spim_init successfull\n");
    }

    pattern[0] = 0;
    pattern[sizeof(pattern)] = 0;

    for(uint16_t pixel = 0; pixel < WS2812_NUM_PIXELS; pixel++) {
        ws2812_set_pixel(pixel, 0, 0 ,0);
    }

    ws2812_write();

    return 0;
}