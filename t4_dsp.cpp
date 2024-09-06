/*
  TFT/VGA driver
  DMA TFT driver based on C64 ILI9341 dma driver from Frank Bösing, 2017
*/

#include "T4_DSP.h"

#include <SPI.h>
#include <DMAChannel.h>

#include "hwcfg.h"

// TFT constants and variables
#define DMA_LINES_PER_BLOCK 64
#define DMA_NUM_SETTINGS 4

#define TFT_SWRESET 0x01
#define TFT_SLPOUT 0x11
#define TFT_INVON 0x21
#define TFT_DISPOFF 0x28
#define TFT_DISPON 0x29
#define TFT_CASET 0x2A
#define TFT_PASET 0x2B
#define TFT_RAMWR 0x2C
#define TFT_MADCTL 0x36
#define TFT_PIXFMT 0x3A
#define TFT_MADCTL_MY 0x80
#define TFT_MADCTL_MX 0x40
#define TFT_MADCTL_MV 0x20
#define TFT_MADCTL_ML 0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH 0x04

#define ST7789PIN_LCD_RST_DELAY 120     ///< delay ms wait for reset finish
#define ST7789_SLPIN_DELAY 120   ///< delay ms wait for sleep in finish
#define ST7789_SLPOUT_DELAY 120  ///< delay ms wait for sleep out finish

#define ST7789_NOP 0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID 0x04
#define ST7789_RDDST 0x09

#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_PTLON 0x12
#define ST7789_NORON 0x13

#define ST7789_INVOFF 0x20
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29

#define ST7789_CASET 0x2A
#define ST7789_PASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_RAMRD 0x2E

#define ST7789_PTLAR 0x30
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36

#define ST7789_MADCTL_MY 0x80
#define ST7789_MADCTL_MX 0x40
#define ST7789_MADCTL_MV 0x20
#define ST7789_MADCTL_ML 0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1 0xDA
#define ST7789_RDID2 0xDB
#define ST7789_RDID3 0xDC
#define ST7789_RDID4 0xDD

#define ST7789_RAMCTRL 0xB0
#define ST7789_PORCTRL 0xB2
#define ST7789_GCTRL 0xB7

#define ST7789_VCOMS 0xbb

#define ST7789_PVGAMCTRL 0xe0
#define ST7789_NVGAMCTRL 0xe1

// LPSPI4 = SPI0 in Teensy 4.0
// LPSPI3 = SPI1 in Teensy 4.0
// LPSPI1 = SPI2 in Teensy 4.0 (used for SD on T4.0 but not T4.1)

#define TFTSPI1
#ifdef TFTSPI1
#define SPI SPI1
#define LPSPIP_TDR LPSPI3_TDR
#define LPSPIP_CR LPSPI3_CR
#define LPSPIP_CFGR1 LPSPI3_CFGR1
#define LPSPIP_TCR LPSPI3_TCR
#define LPSPIP_DER LPSPI3_DER
#define DMAMUX_SOURCE_LPSPIP_TX DMAMUX_SOURCE_LPSPI3_TX
#else
#define LPSPIP_TDR LPSPI4_TDR
#define LPSPIP_CR LPSPI4_CR
#define LPSPIP_CFGR1 LPSPI4_CFGR1
#define LPSPIP_TCR LPSPI4_TCR
#define LPSPIP_DER LPSPI4_DER
#define DMAMUX_SOURCE_LPSPIP_TX DMAMUX_SOURCE_LPSPI4_TX
#endif

#define SPICLOCK LCD_SPI_SPEED
#define SPI_MODE SPI_MODE0

#define TFT_BPL 336  // actucal bits per transmitted line

// VGA constants and macros
#define VGA_RGB(r, g, b) ((((r >> 5) & 0x07) << 5) | (((g >> 5) & 0x07) << 2) | (((b >> 6) & 0x3) << 0))

static DMAChannel dmatx;
volatile uint16_t *tft_buffer;
static int tft_width;
static int tft_height;

#define DELAY_MASK 0x80
#if 0
PROGMEM static const uint8_t init_commands[] = { 
  1+DELAY_MASK, TFT_SWRESET,  150,
  1+DELAY_MASK, TFT_SLPOUT,   255,
  2+DELAY_MASK, TFT_PIXFMT, 0x55, 10,
  2,            TFT_MADCTL, TFT_MADCTL_MV | TFT_MADCTL_BGR, // 270°
//  2, TFT_MADCTL, TFT_MADCTL_MV | TFT_MADCTL_MX | TFT_MADCTL_MY | TFT_MADCTL_BGR, // 90!
//  1, TFT_INVON,
  1, TFT_DISPON,
  0
};
#else
PROGMEM static const uint8_t init_commands[] = {
  1 + DELAY_MASK, TFT_SWRESET, 150,
  1 + DELAY_MASK, TFT_SLPOUT, 150,

  2, TFT_PIXFMT, 0x55,  // 3: Set color mode, 16-bit color
  2, 0x36, 0x00,

  3, 0xB0, 0x00, 0xF0,  // 0xF0 MSB first, 0xF8 LSB first

  6, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,

  2, 0xB7, 0x35,
  2, 0xBB, 0x19,
  2, 0xC0, 0x2C,
  2, 0xC2, 0x01,
  2, 0xC3, 0x12,
  2, 0xC4, 0x20,
  2, 0xC6, 0x0F,

  3, 0xD0, 0xA4, 0xA1,

  15, 0xE0,
  0b11110000,  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
  0b00001001,  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
  0b00010011,  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
  0b00010010,  //     0,     0,     0,  V4P4,  V4P3,  V4P2,  V4P1,  V4P0
  0b00010010,  //     0,     0,     0,  V6P4,  V6P3,  V6P2,  V6P1,  V6P0
  0b00101011,  //     0,     0,  J0P1,  J0P0, V13P3, V13P2, V13P1, V13P0
  0b00111100,  //     0, V20P6, V20P5, V20P4, V20P3, V20P2, V20P1, V20P0
  0b01000100,  //     0, V36P2, V36P1, V36P0,     0, V27P2, V27P1, V27P0
  0b01001011,  //     0, V43P6, V43P5, V43P4, V43P3, V43P2, V43P1, V43P0
  0b00011011,  //     0,     0,  J1P1,  J1P0, V50P3, V50P2, V50P1, V50P0
  0b00011000,  //     0,     0,     0, V57P4, V57P3, V57P2, V57P1, V57P0
  0b00010111,  //     0,     0,     0, V59P4, V59P3, V59P2, V59P1, V59P0
  0b00011101,  //     0,     0, V61P5, V61P4, V61P3, V61P2, V61P1, V61P0
  0b00100001,  //     0,     0, V62P5, V62P4, V62P3, V62P2, V62P1, V62P0

  15, 0XE1,
  0b11110000,  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
  0b00001001,  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
  0b00010011,  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
  0b00001100,  //     0,     0,     0,  V4N4,  V4N3,  V4N2,  V4N1,  V4N0
  0b00001101,  //     0,     0,     0,  V6N4,  V6N3,  V6N2,  V6N1,  V6N0
  0b00100111,  //     0,     0,  J0N1,  J0N0, V13N3, V13N2, V13N1, V13N0
  0b00111011,  //     0, V20N6, V20N5, V20N4, V20N3, V20N2, V20N1, V20N0
  0b01000100,  //     0, V36N2, V36N1, V36N0,     0, V27N2, V27N1, V27N0
  0b01001101,  //     0, V43N6, V43N5, V43N4, V43N3, V43N2, V43N1, V43N0
  0b00001011,  //     0,     0,  J1N1,  J1N0, V50N3, V50N2, V50N1, V50N0
  0b00010111,  //     0,     0,     0, V57N4, V57N3, V57N2, V57N1, V57N0
  0b00010111,  //     0,     0,     0, V59N4, V59N3, V59N2, V59N1, V59N0
  0b00011101,  //     0,     0, V61N5, V61N4, V61N3, V61N2, V61N1, V61N0
  0b00100001,  //     0,     0, V62N5, V62N4, V62N3, V62N2, V62N1, V62N0

  1, 0x13,        // 4: Normal display on, no args, w/delay
  1, TFT_DISPON,  // 5: Main screen turn on, no args, w/delay
  0
};
#endif

uint16_t internal_line_buffer[3][TFT_BPL/2];
volatile uint8_t internal_cur_buf_write = 0;
volatile uint16_t scanline = 0;
volatile uint8_t tft_ready = 1;

void spi_send_byte(uint8_t val) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_LCD_SCK, LOW);  // falling
    //delay(1);
    if (val & 0x80)
      digitalWrite(PIN_LCD_MOSI, HIGH);
    else
      digitalWrite(PIN_LCD_MOSI, LOW);

    val <<= 1;
    digitalWrite(PIN_LCD_SCK, HIGH);  // rising
    //delay(1);
  }
}

void lcd_send_dat_byte(uint8_t val) {
  spi_send_byte(val);
}

void lcd_send_cmd_byte(uint8_t val) {
  digitalWrite(PIN_LCD_DC, LOW);  // command
  spi_send_byte(val);
  digitalWrite(PIN_LCD_DC, HIGH);  // data
}

void T4_DSP::setArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {

  uint8_t coordinates[8] = {
    (uint8_t)(x1 >> 8),
    (uint8_t)(x1 & 0xff),
    (uint8_t)(x2 >> 8),
    (uint8_t)(x2 & 0xff),
    (uint8_t)(y1 >> 8),
    (uint8_t)(y1 & 0xff),
    (uint8_t)(y2 >> 8),
    (uint8_t)(y2 & 0xff)
  };

  lcd_send_cmd_byte(ST7789_CASET);  // column address set
  for (int i = 0; i < 4; i++)
    lcd_send_dat_byte(coordinates[i]);

  lcd_send_cmd_byte(ST7789_PASET);  // page address set
  for (int i = 0; i < 4; i++)
    lcd_send_dat_byte(coordinates[i + 4]);

  lcd_send_cmd_byte(ST7789_RAMWR);  //memory write
}

FASTRUN void TFT_isr(void) {
  // clear IRQ
  DMA_CINT = dmatx.channel;

  // disable tx
  DMA_CERQ = dmatx.channel;
  LPSPIP_CR &= ~LPSPI_CR_MEN;           //disable LPSPI:

  // no more buffers to scanout
  if (scanline == TFT_HEIGHT) {
    tft_ready = 1;
    scanline = 0;
    return;
  }

  // swap bytes
  uint8_t* internal_line_buffer_8_dst = (uint8_t*) &internal_line_buffer[0][0];
  uint8_t* internal_line_buffer_8_src = (uint8_t*) &internal_line_buffer[internal_cur_buf_write][0];

  for (int i = 0; i < TFT_BPL; i += 2) {
    internal_line_buffer_8_dst[i] = internal_line_buffer_8_src[i+1];
    internal_line_buffer_8_dst[i+1] = internal_line_buffer_8_src[i];
  }

  arm_dcache_flush(internal_line_buffer, TFT_BPL);
  // enable tx
  DMA_SERQ = dmatx.channel;
  LPSPIP_CR |= LPSPI_CR_MEN;    //enable LPSPI:

  internal_cur_buf_write++;
  if (internal_cur_buf_write >= 3)
   internal_cur_buf_write = 1;

  uint32_t cnt = 0;
  internal_line_buffer_8_dst = (uint8_t*) &internal_line_buffer[internal_cur_buf_write][0];

  for (int i = 0; i < TFT_WIDTH; i += 2) {
    // 1st pixel
    uint16_t col = tft_buffer[(TFT_WIDTH - i - 1) * TFT_HEIGHT + (TFT_HEIGHT - scanline - 1)];
    uint8_t cred1 = ((col >> 11) & 0x1f) / 2;
    uint8_t cgreen1 = ((col >> 5) & 0x3f) / 4;
    uint8_t cblue1 = ((col & 0x1f)) / 2;

    // 2nd pixel
    col = tft_buffer[(TFT_WIDTH - i - 2) * TFT_HEIGHT + (TFT_HEIGHT - scanline - 1)];
    uint8_t cred2 = ((col >> 11) & 0x1f) / 2;
    uint8_t cgreen2 = ((col >> 5) & 0x3f) / 4;
    uint8_t cblue2 = ((col & 0x1f)) / 2;
    
    // 1st and 2nd pixels packed
    internal_line_buffer_8_dst[cnt] = cred1 * 16 + cgreen1;
    cnt++;      
    internal_line_buffer_8_dst[cnt] = cblue1 * 16 + cred2;
    cnt++;      
    internal_line_buffer_8_dst[cnt] = cgreen2 * 16 + cblue2;
    cnt++;      
  }

  scanline++;

}

void T4_DSP::sendBuffer(uint16_t* buffer) {
  while (!tft_ready);
  tft_ready = 0;
  tft_buffer = buffer;
  TFT_isr();
}

static void T4_DSP::setTftBuffer(uint16_t *buffer) {
  tft_buffer = buffer;
}

T4_DSP::T4_DSP() {
  pinMode(PIN_LCD_DC, OUTPUT);
  digitalWrite(PIN_LCD_DC, 1);
}


// display VGA image
void T4_DSP::begin(void) {
  pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_SCK, OUTPUT);
  pinMode(PIN_LCD_MOSI, OUTPUT);

  for (int l = 0; l < 2; l++) {

    pinMode(PIN_LCD_RST, OUTPUT);
    digitalWrite(PIN_LCD_RST, LOW);
    delay(1);
    digitalWrite(PIN_LCD_RST, HIGH);
    delay(120);

    lcd_send_cmd_byte(ST7789_SLPOUT);  // 2: Out of sleep mode, no args, w/delay
    delay(ST7789_SLPOUT_DELAY);
  }

  lcd_send_cmd_byte(ST7789_COLMOD);  // pixel color format
  lcd_send_dat_byte(0x33);  // 0x33: 12 bit RRRRGGGGBBBB
  lcd_send_cmd_byte(0x36);
  lcd_send_dat_byte(0x00);

  lcd_send_cmd_byte(0xb0);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0xF0);  // fcipaq C0?

  lcd_send_cmd_byte(0xb2);
  lcd_send_dat_byte(0x0C);
  lcd_send_dat_byte(0x0C);
  lcd_send_dat_byte(0x00);
  lcd_send_dat_byte(0x33);
  lcd_send_dat_byte(0x33);

  lcd_send_cmd_byte(0xb7);
  lcd_send_dat_byte(0x35);
  lcd_send_cmd_byte(0xbb);
  lcd_send_dat_byte(0x19);
  lcd_send_cmd_byte(0xC0);
  lcd_send_dat_byte(0x2C);
  lcd_send_cmd_byte(0xC2);
  lcd_send_dat_byte(0x01);
  lcd_send_cmd_byte(0xC3);
  lcd_send_dat_byte(0x12);
  lcd_send_cmd_byte(0xC4);
  lcd_send_dat_byte(0x20);
  lcd_send_cmd_byte(0xC6);
  lcd_send_dat_byte(0x0F);

  lcd_send_cmd_byte(0xD0);
  lcd_send_dat_byte(0xA4);
  lcd_send_dat_byte(0xA1);

  lcd_send_cmd_byte(0xe0);
  lcd_send_dat_byte(0b11110000);  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
  lcd_send_dat_byte(0b00001001);  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
  lcd_send_dat_byte(0b00010011);  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
  lcd_send_dat_byte(0b00010010);  //     0,     0,     0,  V4P4,  V4P3,  V4P2,  V4P1,  V4P0
  lcd_send_dat_byte(0b00010010);  //     0,     0,     0,  V6P4,  V6P3,  V6P2,  V6P1,  V6P0
  lcd_send_dat_byte(0b00101011);  //     0,     0,  J0P1,  J0P0, V13P3, V13P2, V13P1, V13P0
  lcd_send_dat_byte(0b00111100);  //     0, V20P6, V20P5, V20P4, V20P3, V20P2, V20P1, V20P0
  lcd_send_dat_byte(0b01000100);  //     0, V36P2, V36P1, V36P0,     0, V27P2, V27P1, V27P0
  lcd_send_dat_byte(0b01001011);  //     0, V43P6, V43P5, V43P4, V43P3, V43P2, V43P1, V43P0
  lcd_send_dat_byte(0b00011011);  //     0,     0,  J1P1,  J1P0, V50P3, V50P2, V50P1, V50P0
  lcd_send_dat_byte(0b00011000);  //     0,     0,     0, V57P4, V57P3, V57P2, V57P1, V57P0
  lcd_send_dat_byte(0b00010111);  //     0,     0,     0, V59P4, V59P3, V59P2, V59P1, V59P0
  lcd_send_dat_byte(0b00011101);  //     0,     0, V61P5, V61P4, V61P3, V61P2, V61P1, V61P0
  lcd_send_dat_byte(0b00100001);  //     0,     0, V62P5, V62P4, V62P3, V62P2, V62P1, V62P0

  lcd_send_cmd_byte(0xe1);
  lcd_send_dat_byte(0b11110000);  // V63P3, V63P2, V63P1, V63P0,  V0P3,  V0P2,  V0P1,  V0P0
  lcd_send_dat_byte(0b00001001);  //     0,     0,  V1P5,  V1P4,  V1P3,  V1P2,  V1P1,  V1P0
  lcd_send_dat_byte(0b00010011);  //     0,     0,  V2P5,  V2P4,  V2P3,  V2P2,  V2P1,  V2P0
  lcd_send_dat_byte(0b00001100);  //     0,     0,     0,  V4N4,  V4N3,  V4N2,  V4N1,  V4N0
  lcd_send_dat_byte(0b00001101);  //     0,     0,     0,  V6N4,  V6N3,  V6N2,  V6N1,  V6N0
  lcd_send_dat_byte(0b00100111);  //     0,     0,  J0N1,  J0N0, V13N3, V13N2, V13N1, V13N0
  lcd_send_dat_byte(0b00111011);  //     0, V20N6, V20N5, V20N4, V20N3, V20N2, V20N1, V20N0
  lcd_send_dat_byte(0b01000100);  //     0, V36N2, V36N1, V36N0,     0, V27N2, V27N1, V27N0
  lcd_send_dat_byte(0b01001101);  //     0, V43N6, V43N5, V43N4, V43N3, V43N2, V43N1, V43N0
  lcd_send_dat_byte(0b00001011);  //     0,     0,  J1N1,  J1N0, V50N3, V50N2, V50N1, V50N0
  lcd_send_dat_byte(0b00010111);  //     0,     0,     0, V57N4, V57N3, V57N2, V57N1, V57N0
  lcd_send_dat_byte(0b00010111);  //     0,     0,     0, V59N4, V59N3, V59N2, V59N1, V59N0
  lcd_send_dat_byte(0b00011101);  //     0,     0, V61N5, V61N4, V61N3, V61N2, V61N1, V61N0
  lcd_send_dat_byte(0b00100001);  //     0,     0, V62N5, V62N4, V62N3, V62N2, V62N1, V62N0

  lcd_send_cmd_byte(0x35);  // tearing on
  lcd_send_dat_byte(0x00);  // Mode 0: trigger once a frame

/*
  lcd_send_cmd_byte(0x44);  // tearing scan line
  lcd_send_dat_byte(1);  // scanline 128...
  lcd_send_dat_byte(60);  // ... that is somewhere in the middle
*/

  lcd_send_cmd_byte(0xc6);  // frame rate control
  lcd_send_dat_byte(0x0f);   // 60 Hz
  //lcd_send_dat_byte(0x15);   // 50 Hz

  lcd_send_cmd_byte(ST7789_INVON);
  lcd_send_cmd_byte(ST7789_NORON);  // 4: Normal display on, no args, w/delay

  delay(10);

  lcd_send_cmd_byte(ST7789_DISPON);  //Display on

  lcd_send_cmd_byte(ST7789_MADCTL);                                            // Memory Access Control
  lcd_send_dat_byte(ST7789_MADCTL_MX | ST7789_MADCTL_RGB); // Rotation 0 (portrait mode)
  setArea(0, 0, TFT_REALWIDTH - 1, TFT_REALHEIGHT - 1);

    /*
    lcd_send_cmd_byte(ST7789_MADCTL);    // Memory Access Control
    
    #define LCD_ROTATION 0
    #if LCD_ROTATION == 0
    lcd_send_dat_byte(ST7789_MADCTL_MX | ST7789_MADCTL_RGB); // Rotation 0 (portrait mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_WIDTH - 1, PHYS_SCREEN_HEIGHT - 1);
    #elif LCD_ROTATION==1
      lcd_send_dat_byte(ST7789_MADCTL_MV | ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB); // Rotation 90 (landscape mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_HEIGHT - 1, PHYS_SCREEN_WIDTH - 1);
    #elif LCD_ROTATION==2
      lcd_send_dat_byte(ST7789_MADCTL_MY | ST7789_MADCTL_RGB); // Rotation 180 (portrait mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_WIDTH - 1, PHYS_SCREEN_HEIGHT - 1);
    #elif LCD_ROTATION==3
      lcd_send_dat_byte(ST7789_MADCTL_MV | ST7789_MADCTL_RGB); // Rotation 270 (landscape mode)
    lcd_set_addr(0, 0, PHYS_SCREEN_HEIGHT - 1, PHYS_SCREEN_WIDTH - 1);
    #endif
    */

  for (long i = 0; i < 320 * 240 * 2; i++)
    lcd_send_dat_byte(0x00);

}

void T4_DSP::startRefresh(void) {
  tft_width = TFT_WIDTH;
  tft_height = TFT_HEIGHT;

  setArea((TFT_REALWIDTH - TFT_WIDTH) / 2, (TFT_REALHEIGHT - TFT_HEIGHT) / 2, (TFT_REALWIDTH - TFT_WIDTH) / 2 + TFT_WIDTH - 1, (TFT_REALHEIGHT - TFT_HEIGHT) / 2 + TFT_HEIGHT - 1);
  
  // WA for off by one error
  // TODO: fixme
  for (uint32_t i = 0; i < TFT_BPL * (TFT_HEIGHT - 1); i++)
    lcd_send_dat_byte(0x00);

  SPI.setMOSI(PIN_LCD_MOSI);
  SPI.setSCK(PIN_LCD_SCK);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE0));

  dmatx.begin(true);  //false??
  dmatx.disable();
  dmatx.attachInterrupt(TFT_isr);
  dmatx.sourceBuffer((uint16_t*) internal_line_buffer, TFT_BPL);
  //dmatx.TCD->ATTR_SRC = 1;
  dmatx.destination((uint16_t &) LPSPIP_TDR);
 // dmatx.TCD->ATTR_DST = 1;
  dmatx.interruptAtCompletion();
  //dmatx.transferSize(1);
  //dmatx.transferCount(1);

  dmatx.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPIP_TX);

  LPSPIP_CR &= ~LPSPI_CR_MEN;           //disable LPSPI:
  LPSPIP_CFGR1 |= LPSPI_CFGR1_NOSTALL;  //prevent stall from RX
  LPSPIP_TCR = 15;                      // Framesize 16 Bits
  //LPSPIP_FCR = 0; // Fifo Watermark
  LPSPIP_DER = LPSPI_DER_TDDE;  //TX DMA Request Enable
}
