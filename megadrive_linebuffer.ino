/*
Gwenesis : Genesis & megadrive Emulator.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.

__author__ = "bzhxx"
__contact__ = "https://github.com/bzhxx"
__license__ = "GPLv3"

Brought to Teensy 4.1 by fcipaq (daniel.kammer@web.de)

*/
#pragma GCC optimize("Ofast")
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>  // itoa()d

extern "C" {
/* Gwenesis Emulator */
#include "src/gwenesis/cpus/M68K/m68k.h"
#include "src/gwenesis/sound/z80inst.h"
#include "src/gwenesis/sound/ym2612.h"
#include "src/gwenesis/sound/gwenesis_sn76489.h"
#include "src/gwenesis/bus/gwenesis_bus.h"
#include "src/gwenesis/io/gwenesis_io.h"
#include "src/gwenesis/vdp/gwenesis_vdp.h"
#include "src/gwenesis/savestate/gwenesis_savestate.h"
}

#include "controls.h"
#include "hwcfg.h"

#define ROM_HEADER_FILE
#ifdef ROM_HEADER_FILE
#include "rom_vdptest.h"
#endif

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/

#define TFT_ENABLED
#ifdef TFT_ENABLED
#include "t4_dsp.h"
T4_DSP tft;
#endif

uint16_t* fb[2]; 
uint8_t cur_fb = 0;

/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

uint16_t lcd_screen_width = 320;
uint16_t lcd_screen_height = 240;

uint16_t* screen_buf;

extern unsigned char* M68K_RAM; // 68K RAM 
extern unsigned char* VRAM;

/* Clocks and synchronization */
/* system clock is video clock */
volatile int system_clock;

/* shared variables with gwenesis_sn76589 */
int16_t* gwenesis_sn76489_buffer;  // 888 = NTSC, PAL = 1056 (too big) //GWENESIS_AUDIO_BUFFER_LENGTH_PAL];
int sn76489_index;                 /* sn78649 audio buffer index */
int sn76489_clock;                 /* sn78649 clock in video clock resolution */

/* shared variables with gwenesis_ym2612 */
int16_t* gwenesis_ym2612_buffer;  //GWENESIS_AUDIO_BUFFER_LENGTH_PAL];
int ym2612_index;                /* ym2612 audio buffer index */
int ym2612_clock;                /* ym2612 clock in video clock resolution */

#define NUM_AUDIO_BUF 2
// needs to be allocated at compile time, because the not tighly coupled
// RAM somehow is not (properly) accessible from ISR (when interrupts are off)
uint16_t audio_buf[NUM_AUDIO_BUF][GWENESIS_AUDIO_BUFFER_LENGTH_PAL];
volatile uint8_t cur_audio_buf = 0;
volatile int play_buf = 0;
volatile int play_index = 0;
volatile int play_len[NUM_AUDIO_BUF] = {0, 0};

IntervalTimer sndTimer;

/* Configurable keys mapping for A,B and C */
extern unsigned short button_state[3];

#define NB_OF_COMBO 6

uint32_t longpress_s_timer = 0;
uint8_t toggle_s = 0;
int snd_output_volume = 3;
int brightness = 512;  // max = 1024

uint8_t audio_enabled = 1;

/* callback used by the meluator to capture keys */
void gwenesis_io_get_buttons() {

#if 1
  uint16_t dpad = ctrl_dpad_state();
  uint16_t buts = ctrl_button_state();

   button_state[0] = ((dpad & DPAD_LEFT) != 0) << PAD_LEFT |
                     ((dpad & DPAD_RIGHT) != 0) << PAD_RIGHT |
                     ((dpad & DPAD_UP) != 0) << PAD_UP |
                     ((dpad & DPAD_DOWN) != 0) << PAD_DOWN;
  
  if ( ((buts & BUTTON_1) != 0) &&
       ((buts & BUTTON_2) != 0) && 
       ((buts & BUTTON_3) != 0) ) {
    button_state[0] |= 1 << PAD_S;
  } else {
    button_state[0] |= 
//    ((buts & BUTTON_YELLOW) != 0) << PAD_S |
    ((buts & BUTTON_1) != 0) << PAD_A |
    ((buts & BUTTON_2) != 0) << PAD_B |
    ((buts & BUTTON_3) != 0) << PAD_C;
  }

  if ((button_state[0] & (1 << PAD_S)) != 0) {
    if (toggle_s == 0) {
      toggle_s = 1;
      longpress_s_timer = millis();
    }
  } else {
    if (toggle_s == 1) {
      toggle_s = 0;
      if (millis() - longpress_s_timer > 2000) {
        audio_enabled++;
      if (audio_enabled == 2)
        audio_enabled = 0;
      }      
    }
  }
#else
  button_state[0] = 0;
#endif

  button_state[0] = ~button_state[0];
}

static void gwenesis_system_init() {
  /* init emulator sound system with shared audio buffer */

  gwenesis_sn76489_buffer = (int16_t*) malloc(GWENESIS_AUDIO_BUFFER_LENGTH_PAL * 2);
  assert(gwenesis_sn76489_buffer != NULL);
  memset(gwenesis_sn76489_buffer, 0, GWENESIS_AUDIO_BUFFER_LENGTH_PAL * 2);

  gwenesis_ym2612_buffer = (int16_t*) malloc(GWENESIS_AUDIO_BUFFER_LENGTH_PAL * 2);
  assert(gwenesis_ym2612_buffer != NULL);
  memset(gwenesis_ym2612_buffer, 0, GWENESIS_AUDIO_BUFFER_LENGTH_PAL * 2);

}

static void gwenesis_system_shutdown() {
  free((void*)gwenesis_sn76489_buffer);
  free((void*)gwenesis_ym2612_buffer);
}

/************************ Debug function in overlay END ********************************/
/**************************** */

unsigned int lines_per_frame = LINES_PER_FRAME_NTSC;  //262; /* NTSC: 262, PAL: 313 */
unsigned int scan_line;

/*

static bool gwenesis_system_SaveState(char *pathName) {
  int size = 0;
  printf("Saving state...\n");
  size = saveGwenesisState((unsigned char *)ACTIVE_FILE->save_address,ACTIVE_FILE->save_size);
  printf("Saved state size:%d\n", size);
  return true;
}

static bool gwenesis_system_LoadState(char *pathName) {
  printf("Loading state...\n");
  return loadGwenesisState((unsigned char *)ACTIVE_FILE->save_address);
}
*/

FASTRUN void sound_output(void) {

  if (play_index >= play_len[play_buf])
    return;

  if (audio_enabled)
    analogWrite(PIN_SND, audio_buf[play_buf][play_index] / 8);
  else
    analogWrite(PIN_SND, 0); // mute
  
  play_index++;

}

void setup() {
}

DMAChannel dma;

FASTRUN void SND_ISR() {
  dma.disable();
}

/* Main */
void loop() {
  /****************************/
  /*         Controls         */
  /****************************/
  pinMode(PIN_PWR_BTN, INPUT_PULLUP);  // enable
  pinMode(PIN_CHG_CPL, INPUT);  // charge finish
  pinMode(PIN_BAT_VOL, INPUT);  // battery voltage

  // SD
  pinMode(PIN_SD_MOSI, INPUT);  // MOSI
  pinMode(PIN_SD_MISO, INPUT);  // MISO
  pinMode(PIN_SD_SCK, INPUT);  // SCK

  // LCD
  pinMode(PIN_LCD_TE, INPUT);  // TE line
  pinMode(PIN_LCD_MISO, INPUT);  // MISO 

  ctrl_init();

  Serial.begin(115200);
   
  /****************************/
  /*     Graphics output      */
  /****************************/
  screen_buf = (uint16_t*) malloc(lcd_screen_width * 2);
  assert(screen_buf);

  for (int i = 0; i < 2; i++) {
    fb[i] = (uint16_t*) malloc(lcd_screen_width * lcd_screen_height * 2);
    assert(fb[i]);
  }

#ifdef TFT_ENABLED
  tft.begin();
  tft.setTftBuffer(screen_buf);
  tft.startRefresh(); 
#endif

  /****************************/
  /*       Sound output       */
  /****************************/
  pinMode(PIN_SND, OUTPUT);
  // "ideal" frequency: CPU freq / 2 ^ resolution / n
  analogWriteFrequency(PIN_SND, 146484); // "ideal" (sic!) freq for 10 bits --> 4 samples/cycle @ 600 MHz
  analogWriteResolution(10);
  analogWrite(PIN_SND, 0);
  sndTimer.begin(sound_output, 1000000. / 888. / 60.0);  // 888 samples * 60 FPS => 18.7 us per sample

/* LCD */
  pinMode(PIN_LCD_BL_PWM, OUTPUT);
  //digitalWrite(PIN_LCD_BL_PWM, HIGH);
  analogWriteFrequency(PIN_LCD_BL_PWM, 128906);  // "ideal" freq for 10 bits (according to PJRC docs)
  analogWrite(PIN_LCD_BL_PWM, brightness);

/* LCD END */

/* Load ROM  */
#ifdef ROM_HEADER_FILE
  ROM_DATA = ROM_DATA_FLASH;
#else
  ROM_DATA = (const unsigned char*)0x100c3500;  // 800 KB program, the rest is data
#endif

  // emulator init
  M68K_RAM = (unsigned char*) malloc(0x10000);
  assert(M68K_RAM);
  VRAM = (unsigned char*) malloc(0x10000);
  assert(VRAM);

  load_cartridge();
  gwenesis_system_init();
  power_on();
  reset_emulation();

  gwenesis_vdp_set_buffer(screen_buf);

  int32_t frame_delay = 0;

  extern unsigned char gwenesis_vdp_regs[0x20];
  extern unsigned int gwenesis_vdp_status;
  extern unsigned int screen_width, screen_height;
  static int vert_screen_offset = REG1_PAL ? 0 : (240 - 224);
  int hint_counter;

  extern int hint_pending;

  uint32_t frame_timer_start = micros();
  uint32_t frame_timer_end;
  uint32_t fps_timer = millis();
  int fps_cnt = 0;

  /****************************/
  /*       Sound output       */
  /****************************/

//#define DEBUG_DISPLAY
#ifdef DEBUG_DISPLAY
  char debug_print[11];
  uint32_t t0;
  uint16_t cnt = 0;
  t0 = millis();
#endif

  while (true) {

    /* Eumulator loop */
    hint_counter = gwenesis_vdp_regs[10];

    screen_height = REG1_PAL ? 240 : 224;
    screen_width = REG12_MODE_H40 ? 320 : 256;
    lines_per_frame = REG1_PAL ? LINES_PER_FRAME_PAL : LINES_PER_FRAME_NTSC;
    vert_screen_offset = REG1_PAL ? 0 : (240 - 224);

    gwenesis_vdp_render_config();

    /* Reset the difference clocks and audio index */
    system_clock = 0;
    zclk = 0;

    ym2612_clock = 0;
    ym2612_index = 0;

    sn76489_clock = 0;
    sn76489_index = 0;

    scan_line = 0;

    // timer
    frame_timer_start = micros();

    while (scan_line < lines_per_frame) {
      /* CPUs  */
      m68k_run(system_clock + VDP_CYCLES_PER_LINE);
      z80_run(system_clock + VDP_CYCLES_PER_LINE);

      /* Video */
      if (scan_line < lcd_screen_height) {
        gwenesis_vdp_render_line(scan_line); /* render scan_line */
        if ((scan_line >= 0) && (scan_line < (lcd_screen_height - vert_screen_offset))) {
          //            glcdBlitBuf(210, -scan_line + 10, scr_overlay_buf, screen, 0x0001);
#ifdef TFT_ENABLED
          for (int i = 0; i < lcd_screen_width; i++)
            fb[cur_fb][i + scan_line * lcd_screen_width] = screen_buf[i];
#endif
        }
      }


      // On these lines, the line counter interrupt is reloaded
      if ((scan_line == 0) || (scan_line > screen_height)) {
        //  if (REG0_LINE_INTERRUPT != 0)
        //    printf("HINTERRUPT counter reloaded: (scan_line: %d, new
        //    counter: %d)\n", scan_line, REG10_LINE_COUNTER);
        hint_counter = REG10_LINE_COUNTER;
      }

      // interrupt line counter
      if (--hint_counter < 0) {
        if ((REG0_LINE_INTERRUPT != 0) && (scan_line <= screen_height)) {
          hint_pending = 1;
          // printf("Line int pending %d\n",scan_line);
          if ((gwenesis_vdp_status & STATUS_VIRQPENDING) == 0)
            m68k_update_irq(4);
        }
        hint_counter = REG10_LINE_COUNTER;
      }

      scan_line++;

      // vblank begin at the end of last rendered line
      if (scan_line == screen_height) {

        if (REG1_VBLANK_INTERRUPT != 0) {
          // printf("IRQ VBLANK\n");
          gwenesis_vdp_status |= STATUS_VIRQPENDING;
          m68k_set_irq(6);
        }
        z80_irq_line(1);
      }

      if (scan_line == (screen_height + 1)) {
        z80_irq_line(0);
      }

      system_clock += VDP_CYCLES_PER_LINE;
    }

    /*************************/
    /*        AUDIO          */  
    /*************************/
    // generate missing samples
    gwenesis_SN76489_run(system_clock);
    ym2612_run(system_clock);

    uint16_t* cur_buf = audio_buf[cur_audio_buf];
    for (int h = 0; h < ym2612_index; h++)
      cur_buf[h] = ((gwenesis_ym2612_buffer[h] + gwenesis_sn76489_buffer[h]) / 2 + 32767) / 64;

    play_len[cur_audio_buf] = ym2612_index;

    /*************************/
    /*       SYNC A/V        */  
    /*************************/
    // sync a/v to LCD scanline
    while (digitalRead(PIN_LCD_TE) == LOW);

    // set audio buffer for scan out
    play_index = 0;
    play_buf = cur_audio_buf;

    cur_audio_buf++;
    if (cur_audio_buf >= NUM_AUDIO_BUF)
      cur_audio_buf = 0;
   
    tft.sendBuffer(fb[cur_fb]);

    cur_fb++;
    if (cur_fb >= 2)
      cur_fb = 0;

    if (digitalRead(PIN_PWR_BTN) == LOW) {
      analogWrite(PIN_LCD_BL_PWM, 0);
      analogWrite(PIN_SND, 0);
      while (digitalRead(PIN_PWR_BTN) == LOW);
      analogWrite(PIN_LCD_BL_PWM, brightness);
    }

    /*************************/
    /*         FPS           */  
    /*************************/
    #if 1
    fps_cnt++;
    if (millis() - fps_timer > 3000) {
      Serial.print("Sply voltage (V): ");
      Serial.println((float) analogRead(PIN_BAT_VOL) / 1024. * 3.3 * 2.0);
      if (digitalRead(PIN_CHG_CPL) == HIGH)
        Serial.println("Charging complete (if grid powered).");
      else
        Serial.println("Charging ongoing (if grid powered).");
      Serial.println(fps_cnt * 1000 / (millis() - fps_timer));
      fps_timer = millis();
      fps_cnt = 0;
    }
    #endif

    // reset m68k cycles to the begin of next frame cycle
    m68k.cycles -= system_clock;

  }  // end of emulator loop
}
