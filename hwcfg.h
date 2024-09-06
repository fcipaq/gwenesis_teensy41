/*
 * pplib - a library for the Pico Held handheld
 *
 * Copyright (C) 2023 Daniel Kammer (daniel.kammer@web.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#ifndef HWCFG_H
#define HWCFG_H

// 0 = dev board      (protoss) - no longer exists


#define CONFIG_NO 0

#if CONFIG_NO==0
// ================================== CONFIG 5 ====================================================
/*----------------- buttons assignment --------------- */
#define BUTTON_PULL_MODE INPUT_PULLDOWN
#define BUTTON_PRESSED HIGH

#define PIN_ANALOG_X  38
#define PIN_ANALOG_Y  39

#define PIN_BUTTON_1  15
#define PIN_BUTTON_2  16
#define PIN_BUTTON_3  14

/* ------------------------ Power management pins ------------------------*/
#define PIN_BAT_VOL      17  // bat volume
#define PIN_CHG_CPL      23  // charging complete
#define PIN_PWR_BTN      22
//#define BAT_PIN_SRC   24

/* ------------------------ Sound assignment ------------------------*/
#define PIN_SND       3

/* --------------------------- LCD settings ---------------------------*/
// LCD_SPI_SPEED defines the SPI speed (in MHz)
// ST7789 datasheet states a max. clock frequency of 15 MHz
// Colors turn red above 75 MHz which means too much overclocking... I guess...
#define LCD_SPI_SPEED 70000000

/* ------------------------ LCD pin assignment ------------------------*/
#define PIN_LCD_BL_PWM 5

#define PIN_LCD_TE     6     // tearing pin
#define PIN_LCD_DC     0    // data/command control pin
#define PIN_LCD_SCK   27    // clock
#define PIN_LCD_MOSI  26    // MOSI
#define PIN_LCD_MISO   1    // MISO
#define PIN_LCD_RST   10    // reset pin


// SD pins
#define PIN_SD_MISO       12
#define PIN_SD_MOSI       11
#define PIN_SD_SCK        13
//#define PIN_SD_CS         -1

/* ---------------------- TFT driver ----------------------*/
//#define LCD_DRIVER_ST7789
//#include "lcd_drv/st7789_drv.h"

#endif  // CONFIG SELECTION

#endif //HWCFG_H
