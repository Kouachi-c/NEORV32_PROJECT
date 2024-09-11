//
// Created by ekon-ihc on 9/10/24.
//

#include "Neorv32_BNO055.h"


/**
 * Constructor
 * @param sensorID ID of the sensor
 * @param address UART address of the sensor
 * @param uart UART peripheral used for communication
 */
Neorv32_BNO055::Neorv32_BNO055(int32_t sensorID, uint8_t address, neorv32_uart_t *uart) {
    //std::cout<<"BNO055 constructor\n";

    _sensorID = sensorID;  // ID of the sensor
    _address = address;  // UART address of the sensor
    _uart = uart;  // UART peripheral used for communication


    // Initialize uart0 with no interrupt support
    neorv32_uart0_setup(BAUD_RATE, 0);
}


/**
 * Destructor
 */
Neorv32_BNO055::~Neorv32_BNO055() {
    if (_uart) {
        delete _uart;  // Free allocated memory
        _uart = nullptr;
    }
}
