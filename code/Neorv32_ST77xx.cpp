//
// Created by ekon-ihc on 9/14/24.
//

#include "Neorv32_ST77xx.h"

#define SPI_BAUD_RATE 5000000 // 5 MHz

Neorv32_ST77xx::Neorv32_ST77xx(uint16_t w, uint16_t h, int8_t _CS, int8_t _DC, int8_t _MOSI,
                               int8_t _SCLK, int8_t _RST = -1, int8_t _MISO = -1);
    _width = w;
    _height = h;
    _cs = _CS;
    _dc = _DC;
    _rst = _RST;
    _mosi = _MOSI;
    _sclk = _SCLK;
    _miso = _MISO;
}

Neorv32_ST77xx::Neorv32_ST77xx(uint16_t w, uint16_t h, int8_t CS, int8_t DC, int8_t RST = -1) {
    _width = w;
    _height = h;
    _cs = CS;
    _dc = DC;
    _rst = RST;
    _mosi = 0;
    _sclk = 1;
    _miso = -1;
}

void Neorv32_ST77xx::displayInit(const uint8_t *addr) {
    uint8_t numCommands, cmd, numArgs;*
    uint16_t ms;
    numCommands = pgm_read_byte(addr++); // Number of commands to follow
    while (numCommands) {              // For each command...
        cmd = pgm_read_byte(addr++);       // Read command
        numArgs = pgm_read_byte(addr++);   // Number of args to follow
        ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
        sendCommand(cmd, addr, numArgs);
        addr += numArgs;

        if (ms) {
            ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
            if (ms == 255)
                ms = 500; // If 255, delay for 500 ms
            delay(ms);
        }
        numCommands--;
    }

}

void Neorv32_ST77xx::begin(uint32_t freq) {
    if (!freq) freq = SPI_BAUD_RATE;
    neorv32_spi_setup(freq, 0);
    neorv32_spi_cs_en(_cs);
}

void Neorv32_ST77xx::commonInit(const uint8_t *cmdList) {
    begin();
    if(cmdList) displayInit(cmdList);

}

void Neorv32_ST77xx::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t xa = ((uint16_t)x << 8) + x + w - 1;
    uint16_t ya = ((uint16_t)y << 8) + y + h - 1;
    sendCommand(ST77XX_CASET, &xa, 4);
    sendCommand(ST77XX_RASET, &ya, 4);
    sendCommand(ST77XX_RAMWR, NULL, 0);
}

void Neorv32_ST77xx::setRotation(uint8_t r) {
    uint8_t madctl = 0;
    rotation = r & 3; // Just perform the operation on the last two bits
    switch (rotation) {
        case 0:
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
            _width = _width;
            _height = _height;
            break;
        case 1:
            madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
            _width = _height;
            _height = _width;
            break;
        case 2:
            madctl = ST77XX_MADCTL_RGB;
            _width = _width;
            _height = _height;
            break;
        case 3:
            madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
            _width = _height;
            _height = _width;
            break;
    }
    sendCommand(ST77XX_MADCTL, &madctl, 1);
}

void Neorv32_ST77xx::enableDisplay(boolean enable) {
    sendCommand(enable ? ST77XX_DISPON : ST77XX_DISPOFF, NULL, 0);
}

void Neorv32_ST77xx::setColRowStart(int8_t col, int8_t row) {
    _colstart = col;
    _rowstart = row;
}

void Neorv32_ST77xx::enableTearing(boolean enable) {
    sendCommand(enable ? ST77XX_TEON : ST77XX_TEOFF, NULL, 0);
}

void Neorv32_ST77xx::enableSleep(boolean enable) {
    sendCommand(enable ? ST77XX_SLPIN : ST77XX_SLPOUT, NULL, 0);
}

void Neorv32_ST77xx::sendCommand(uint8_t cmd, const uint8_t *data, uint8_t numDataBytes) {
    neorv32_gpio_pin_clr(_dc);
    neorv32_gpio_pin_clr(_cs);
    neorv32_spi_trans(cmd);
    neorv32_gpio_pin_set(_cs);
    if (data && numDataBytes) {
        neorv32_gpio_pin_set(_dc);
        neorv32_gpio_pin_clr(_cs);
        neorv32_spi_trans(data);
        neorv32_spi_trans(numDataBytes);
        neorv32_gpio_pin_set(_cs);
    }
}