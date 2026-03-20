#pragma once

// --- T-Display-S3 Parallel 8-Bit Configuration ---
#define ST7789_DRIVER
#define INIT_SEQUENCE_3
#define CGRAM_OFFSET
#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_ON
#define TFT_PARALLEL_8_BIT

#define TFT_WIDTH  170
#define TFT_HEIGHT 320

#define TFT_CS   6
#define TFT_DC   7
#define TFT_RST  5
#define TFT_WR   8
#define TFT_RD   9

#define TFT_D0   39
#define TFT_D1   40
#define TFT_D2   41
#define TFT_D3   42
#define TFT_D4   45
#define TFT_D5   46
#define TFT_D6   47
#define TFT_D7   48

//#define TFT_BL   38
//#define TFT_BACKLIGHT_ON 1

#undef TFT_BL
#define TFT_BL 21

#define LOAD_GLCD
#define LOAD_FONT2