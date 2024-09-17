

#include <neorv32.h>
#include "Neorv32_BNO055.h"
#include "Neorv32_ST7789.h"

#define cs 0
#define dc 1
#define rst 2



int main() {
    // Sensor and screen initialization
    Neorv32_ST7789 screen( cs, dc, rst);
    Neorv32_BNO055 bno2(BNO055_ID);

    // Parameters for the sensor
    neorv32_vector_type_t vector = VECTOR_ACCELEROMETER;
    neorv32_bno055_unit_sel_t unitSel = UNIT_SEL_AND_CEl_DPS_DEG_MS2;
    uint8_t numDataBytes = 3;
    int16_t *data = new int16_t[numDataBytes];

    bno2.begin(OPERATION_MODE_AMG, unitSel);

    screen.init(240, 240);
    screen.begin();
    screen.enableDisplay(true);

    while(1) {
        bno2.getVector(vector, data);
        screen.sendCommand(ST77XX_MADCTL, data, numDataBytes);

    }



    // neorv32_uart0_printf("Hello World! %d\n");
    return 0;
}


