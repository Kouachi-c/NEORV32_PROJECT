
//#include "camera.h"
//#include "BNO055.h"
//#include "screen.h"
#include <neorv32.h>
#include "Neorv32_BNO055.h"
#include "Neorv32_ST7789.h"

int8_t test = 1;
uint16_t t = 240;



int main() {
    Neorv32_ST7789 screen( test, test, test);
    Neorv32_BNO055 bno2;
    uint16_t x = 5;
    neorv32_uart0_printf("Hello World! %d\n", x);
    return 0;
}
