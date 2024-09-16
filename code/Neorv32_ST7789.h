//
// Created by ekon-ihc on 9/14/24.
//

#ifndef NEORV32_PROJECT_NEORV32_ST7789_H
#define NEORV32_PROJECT_NEORV32_ST7789_H

#include "Neorv32_ST77xx.h"



class Neorv32_ST7789 : public Neorv32_ST77xx {
    private:
        uint16_t windowWidth;
        uint16_t windowHeight;

    protected:
        uint8_t _colstart2 = 0, _rowstart2 = 0;

    public:
        Neorv32_ST7789(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst);
        Neorv32_ST7789(int8_t cs, int8_t dc, int8_t rst);
        void setRotation(uint8_t m);
        void init(uint16_t width, uint16_t height);



};


#endif //NEORV32_PROJECT_NEORV32_ST7789_H
