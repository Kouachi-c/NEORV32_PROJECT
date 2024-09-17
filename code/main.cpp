
//#include "camera.h"
//#include "BNO055.h"
//#include "screen.h"
#include <neorv32.h>
#include "Neorv32_BNO055.h"
#include "Neorv32_ST7789.h"

#define cs 0
#define dc 1
#define rst 2


int main() {
    Neorv32_ST7789 screen( cs, dc, rst);
    Neorv32_BNO055 bno2(BNO055_ID);
    bno2.begin(OPERATION_MODE_AMG, UNIT_SEL_AND_CEl_DPS_DEG_MS2);
    screen.init(240, 240);
    uint16_t x = 5;
    neorv32_uart0_printf("Hello World! %d\n", x);
    return 0;
}
