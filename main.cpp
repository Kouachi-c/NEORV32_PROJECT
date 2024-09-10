
#include "camera.h"
#include "BNO055.h"
#include "screen.h"
#include <neorv32.h>


int main() {
    camera cam;
    BNO055 bno;
    screen scr;
    uint16_t x = 5;
    //neorv32_uart0_printf("Hello World! %d\n", x);
    return 0;
}
