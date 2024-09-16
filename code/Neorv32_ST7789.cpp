//
// Created by ekon-ihc on 9/14/24.
//

#include "Neorv32_ST7789.h"

Neorv32_ST7789::Neorv32_ST7789(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst)
    : Neorv32_ST77xx(240, 240, cs, dc, mosi, sclk, rst) {
    // Constructor
}

Neorv32_ST7789::Neorv32_ST7789(int8_t cs, int8_t dc, int8_t rst)
    : Neorv32_ST77xx(240, 240, cs, dc, rst) {
    // Constructor
}


static const uint8_t PROGMEM
generic_st7789[] =  {                // Init commands for 7789 screens
9,                              //  9 commands in list:
ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
150,                          //     ~150 ms delay
ST77XX_SLPOUT ,   ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
10,                          //      10 ms delay
ST77XX_COLMOD , 1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
0x55,                         //     16-bit color
10,                           //     10 ms delay
ST77XX_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
0x08,                         //     Row/col addr, bottom-top refresh
ST77XX_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
0x00,
0,        //     XSTART = 0
0,
240,  //     XEND = 240
ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
0x00,
0,             //     YSTART = 0
320>>8,
320&0xFF,  //     YEND = 320
ST77XX_INVON  ,   ST_CMD_DELAY,  //  7: hack
10,
ST77XX_NORON  ,   ST_CMD_DELAY, //  8: Normal display on, no args, w/delay
10,                           //     10 ms delay
ST77XX_DISPON ,   ST_CMD_DELAY, //  9: Main screen turn on, no args, delay
10 };


void Neorv32_ST7789::init(uint16_t width, uint16_t height) {
    _rowstart = 320 - height;
    _colstart = _colstart2 = (240 - width);
    windowWidth = width;
    windowHeight = height;
    Neorv32_ST77xx::displayInit(generic_st7789);
    setRotation(0);
}

void Neorv32_ST7789::setRotation(uint8_t m) {
    uint8_t madctl = 0;
    rotation = m & 3; // can't be higher than 3
    switch (rotation) {
        case 0:
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
            _width = windowWidth;
            _height = windowHeight;
            break;
        case 1:
            madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
            _width = windowHeight;
            _height = windowWidth;
            break;
        case 2:
            madctl = ST77XX_MADCTL_RGB;
            _width = windowWidth;
            _height = windowHeight;
            break;
        case 3:
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
            _width = windowHeight;
            _height = windowWidth;
            break;
    }
    sendCommand(ST77XX_MADCTL, &madctl, 1);
}