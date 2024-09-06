/*
  TFT/VGA driver
  DMA TFT driver based on C64 ILI9341 dma driver from Frank Bösing, 2017
*/

#ifndef _T4_DSPH_
#define _T4_DSPH_

#ifdef __cplusplus
#include <Arduino.h>
#include <DMAChannel.h>
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH      224 
#endif
#define TFT_REALWIDTH  240

#ifndef TFT_HEIGHT
#define TFT_HEIGHT     320
#endif
#define TFT_REALHEIGHT 320

#ifdef __cplusplus

class T4_DSP
{
  public:
    T4_DSP();

    void begin();
    void startRefresh(void);
    
    int get_frame_buffer_size(int *width, int *height);
    void setArea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
    static void setTftBuffer(uint16_t* buffer);
    void sendBuffer(uint16_t* buffer);
  
  protected:   
    //static void TFT_isr(void);
};

#endif
#endif
