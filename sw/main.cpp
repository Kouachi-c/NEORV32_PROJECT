

#include <neorv32.h>
#include "Neorv32_ST7789.h"

#define cs 0
#define dc 1
#define rst 2



int main() {
    // Sensor and screen initialization
    Neorv32_ST7789 screen( cs, dc, rst);

    // Parameters for the sensor
    uint8_t numDataBytes = 3;
    int16_t *data = new int16_t[numDataBytes];

    screen.init(240, 240);
    screen.begin();
    screen.enableDisplay(true);




    // neorv32_uart0_printf("Hello World! %d\n");
    return 0;
}


