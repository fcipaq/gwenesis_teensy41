### This is _Gwenesis_ on the _Pico Held_ (_Teensy 4.1_ edition)

## What is this?

This is a _Sega Megadrive/Genesis_ emulator by [bzhxx](https://github.com/bzhxx/gwenesis) running on the _Teensy 4.1_.

## How to build

Just build with _Arduino_ (I used _Arduino_ 2.3.2 and _Teensy_ core 0.59.3). No PSRAM is required.

## About the software

The _Teensy's_ CPU is very fast and able to run _Genesis_ titles at 60 fps. However, I found the _Teensy's_ capabilities to connect to a LCD quite limited. The _flexio_ allows for flexible output capabilities (well... hence the name) but for an 8 or 16 bit MCU attachment of an LCD some pins were not broken out. So I found no way of connecting a display in such a way (which does not mean that there might be a way, only that I found none).
Connecting the display via SPI is possible, however, the ST7789 I used was not able to achieve 60 fps even at the LCD's maximum supported SPI speed (which already is overclocked by a factor of 2). The ST7789 controller however supports 12 bit (RRRRGGGGBBBB) pixel data transmission. And that is a reduction of 25% in amount of transmitted image data. Together with the overlocked SPI this was just enough to give you 60 fps. The emulator uses a 16 bit framebuffer which is then converted to 12 bits on the fly (just ahead of “the beam”). And it’s synced to the LCD so it’s tearing free.
I know there’s different approaches like a differential/partial buffer transmission but I did not have enough RAM left for such an approach (my _Teensy_ has no PSRAM).
Also the ROMS need to be in byte swapped order (this speeds up the execution).
Please regard this as a proof of concept. No effort was taken to clean up the code nor implement things like save states, reading ROMS from SD etc.


