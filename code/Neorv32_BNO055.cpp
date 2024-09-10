//
// Created by ekon-ihc on 9/10/24.
//

#include "Neorv32_BNO055.h"

Neorv32_BNO055::Neorv32_BNO055(int32_t sensorID, uint8_t address, neorv32_uart_t *uart) {
    //std::cout<<"BNO055 constructor\n";

    _sensorID = sensorID;  // ID of the sensor
    _address = address;  // UART address of the sensor
    _uart = uart;  // UART peripheral used for communication


    // Initialize uart
    neorv32_uart_setup(uart, 115200, PARITY_NONE, FLOW_CONTROL_NONE);
}
