//
// Created by ekon-ihc on 9/14/24.
//

#include "Neorv32_ST77xx.h"

#define SPI_BAUD_RATE 5000000 // 5 MHz

Neorv32_ST77xx::Neorv32_ST77xx(uint16_t w, uint16_t h, int8_t _CS, int8_t _DC, int8_t _MOSI,
                               int8_t _SCLK, int8_t _RST, int8_t _MISO){
    _width = w;
    _height = h;
    _cs = _CS;
    _dc = _DC;
    _rst = _RST;
    _mosi = _MOSI;
    _sclk = _SCLK;
    _miso = _MISO;
}

Neorv32_ST77xx::Neorv32_ST77xx(uint16_t w, uint16_t h, int8_t _CS, int8_t _DC, int8_t _RST) {
    _width = w;
    _height = h;
    _cs = _CS;
    _dc = _DC;
    _rst = _RST;
    _mosi = -1;
    _sclk = -1;
    _miso = -1;
}


void Neorv32_ST77xx::displayInit(const uint8_t *addr) {
    uint8_t numCommands, cmd, numArgs;
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
            neorv32_cpu_delay_ms(ms);
        }
        numCommands--;
    }

}

void Neorv32_ST77xx::begin() {
    int prsc = 5, cdiv = 0, clk_phase = 0, clk_polarity = 0;
    uint32_t irq_mask = 0;
    neorv32_spi_setup(prsc, cdiv, clk_phase, clk_polarity, irq_mask);
    neorv32_spi_cs_en(_cs);
}

void Neorv32_ST77xx::commonInit(const uint8_t *cmdList) {
    begin();
    if(cmdList) displayInit(cmdList);

}

void Neorv32_ST77xx::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint8_t xa = (uint8_t)(x << 8);
    uint8_t ya = (uint8_t)(y << 8) ;
    uint8_t wa = (uint8_t)(0 << 8);
    sendCommand(ST77XX_CASET, &xa, 4);
    sendCommand(ST77XX_RASET, &ya, 4);
    sendCommand(ST77XX_RAMWR, &wa, 0);
}

void Neorv32_ST77xx::setRotation(uint8_t r) {
    uint8_t madctl = 0;
    uint8_t rotation = r & 3; // Just perform the operation on the last two bits
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
    uint8_t wa = (uint8_t)(0 << 8);
    if(enable) {
        sendCommand(ST77XX_DISPON, &wa, 0);
    } else {
        sendCommand(ST77XX_DISPOFF, &wa, 0);
    }
}

void Neorv32_ST77xx::setColRowStart(int8_t col, int8_t row) {
    _colstart = col;
    _rowstart = row;
}

void Neorv32_ST77xx::enableTearing(boolean enable) {
    uint8_t wa = (uint8_t)(0 << 8);
    if(enable) {
        sendCommand(ST77XX_TEON, &wa, 0);
    } else {
        sendCommand(ST77XX_TEOFF, &wa, 0);
    }
}

void Neorv32_ST77xx::enableSleep(boolean enable) {
    uint8_t wa = (uint8_t)(0 << 8);
    if(enable) {
        sendCommand(ST77XX_SLPIN, &wa, 0);
    } else {
        sendCommand(ST77XX_SLPOUT, &wa, 0);
    }
}

void Neorv32_ST77xx::sendCommand(uint8_t cmd, uint8_t *data, uint8_t numDataBytes) {
    neorv32_gpio_pin_set(_dc, 0);
    neorv32_spi_trans(cmd);
    uint8_t length = sizeof(data);
    uint8_t tx_data[length];
    for (uint8_t i = 0; i < length; i++) {
        tx_data[i] = data[i];
    }
    if (data && numDataBytes) {
        neorv32_gpio_pin_set(_dc, 1);
        for (uint8_t i = 0; i < length; i++) {
        neorv32_spi_trans(tx_data[i]);
        }
        neorv32_spi_trans(numDataBytes);
    }
}

void Neorv32_ST77xx::sendCommand(uint8_t cmd, const uint8_t *data, uint8_t numDataBytes) {
    neorv32_gpio_pin_set(_dc, 0);
    neorv32_spi_trans(cmd);
    uint8_t length = sizeof(data);
    uint8_t tx_data[length];
    for (uint8_t i = 0; i < length; i++) {
        tx_data[i] = data[i];
    }
    if (data && numDataBytes) {
        neorv32_gpio_pin_set(_dc, 1);
        for (uint8_t i = 0; i < length; i++) {
            neorv32_spi_trans(tx_data[i]);
        }
        neorv32_spi_trans(numDataBytes);
    }
}

void Neorv32_ST77xx::sendCommand(uint8_t cmd, int16_t *data, uint8_t numDataBytes) {
    neorv32_gpio_pin_set(_dc, 0);
    neorv32_spi_trans(cmd);
    uint8_t length = sizeof(data);
    int16_t tx_data[length];
    for (uint8_t i = 0; i < length; i++) {
        tx_data[i] = data[i];
    }
    if (data && numDataBytes) {
        neorv32_gpio_pin_set(_dc, 1);
        for (uint8_t i = 0; i < length; i++) {
            neorv32_spi_trans(tx_data[i]);
        }
        neorv32_spi_trans(numDataBytes);
    }
}