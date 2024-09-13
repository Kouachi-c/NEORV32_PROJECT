//
// Created by ekon-ihc on 9/10/24.
//

#include "Neorv32_BNO055.h"


/**
 * Constructor
 * @param sensorID ID of the sensor
 * @param irq_mask Interrupt configuration mask (CTRL's irq_* bits).
 */
Neorv32_BNO055::Neorv32_BNO055(int32_t sensorID) {
    //std::cout<<"BNO055 constructor\n";

    _sensorID = sensorID;  // ID of the sensor


    // Initialize uart (uart0 by default) with interrupt (no interrupt by default) support
    neorv32_uart0_setup( BAUD_RATE, 0);




}

/**
 * Initialize the sensor
 * @param mode Operation mode
 *        mode values:
 *        - OPERATION_MODE_CONFIG
 *        - OPERATION_MODE_ACCONLY
 *        - OPERATION_MODE_MAGONLY
 *        - OPERATION_MODE_GYRONLY
 *        - OPERATION_MODE_ACCMAG
 *        - OPERATION_MODE_ACCGYRO
 *        - OPERATION_MODE_MAGGYRO
 *        - OPERATION_MODE_AMG
 *        - OPERATION_MODE_IMUPLUS
 *        - OPERATION_MODE_COMPASS
 *        - OPERATION_MODE_M4G
 *        - OPERATION_MODE_NDOF_FMC_OFF
 *        - OPERATION_MODE_NDOF
 *
 *        - Default mode is OPERATION_MODE_NDOF
 * @param unitSel Output data units
 *       unitSel values:
 *       - UNIT_SEL_AND_CEl_DPS_RAD_MS2
 *       - UNIT_SEL_AND_CEl_RPS_RAD_MS2
 *       - UNIT_SEL_AND_CEl_DPS_DEG_MS2
 *       - UNIT_SEL_AND_CEl_RPS_DEG_MS2
 *       - UNIT_SEL_AND_F_DPS_RAD_MS2
 *       - UNIT_SEL_AND_F_RPS_RAD_MS2
 *       - UNIT_SEL_AND_F_DPS_DEG_MS2
 *       - UNIT_SEL_AND_F_RPS_DEG_MS2
 *       - UNIT_SEL_WIN_CEl_DPS_RAD_MS2
 *       - UNIT_SEL_WIN_CEl_RPS_RAD_MS2
 *       - UNIT_SEL_WIN_CEl_DPS_DEG_MS2
 *       - UNIT_SEL_WIN_CEl_RPS_DEG_MS2
 *       - UNIT_SEL_WIN_F_DPS_RAD_MS2
 *       - UNIT_SEL_WIN_F_RPS_RAD_MS2
 *       - UNIT_SEL_WIN_F_DPS_DEG_MS2
 *       - UNIT_SEL_WIN_F_RPS_DEG_MS2
 *
 *       - Default unitSel is UNIT_SEL_AND_CEl_DPS_DEG_MS2
 * @return True if the process was successfully initialized, false otherwise
 */
bool Neorv32_BNO055::begin(neorv32_bno055_opmode_t mode, neorv32_bno055_unit_sel_t unitSel) {

    // Check if the sensor is the right one
    uint8_t id = receive(BNO055_CHIP_ID_ADDR);  // Read the chip ID
    if(id != BNO055_ID) {   // Check if the chip ID is correct
        neorv32_cpu_delay_ms(1000);  // Wait for 1 second
        id = receive(BNO055_CHIP_ID_ADDR);  // Read the chip ID again
        if(id != BNO055_ID) {  // Check one more time if the chip ID is correct
            return false;  // Return false if the chip ID is incorrect
        }
    }

    // Set the sensor to CONFIG mode
    setMode(mode);
    neorv32_cpu_delay_ms(19);  // Wait for 20 ms (this time delay is for security. any mode to config mode switch requires 19 ms and config mode to any mode switch requires 7 ms.)

    // Reset the sensor
    transmit(BNO055_SYS_TRIGGER_ADDR, 0x20);  // Reset the sensor
    neorv32_cpu_delay_ms(650);  // Wait for 650 ms (this time delay is for security. reset requires 650 ms.)

    while(receive(BNO055_CHIP_ID_ADDR) != BNO055_ID) {  // Check if the chip ID is correct
        neorv32_cpu_delay_ms(1000);  // Wait for 1 second
    }
    neorv32_cpu_delay_ms(50);  // Wait for 50 ms

    // Set the sensor to normal power mode
    transmit(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);  // Set the power mode to normal
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms

    // Select the page 0
    transmit(BNO055_PAGE_ID_ADDR, 0x00);  // Select the page 0 (the output data format register is in page 0)

    // Set output data units
    transmit(BNO055_UNIT_SEL_ADDR, unitSel);  // Set the output data units (default is UNIT_SEL_AND_CEl_DPS_DEG_MS2)

    // Set the sensor SYS_TRIGGER register to 0x00
    transmit(BNO055_SYS_TRIGGER_ADDR, 0x00);  // Set the sensor SYS_TRIGGER register to 0x00

    // Set the sensor to the selected mode
    setMode(mode);
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms

    return true;
}

/**
 * Set the sensor to the selected mode
 * @param mode Operation mode
 *        mode values:
 *        - OPERATION_MODE_CONFIG
 *        - OPERATION_MODE_ACCONLY
 *        - OPERATION_MODE_MAGONLY
 *        - OPERATION_MODE_GYRONLY
 *        - OPERATION_MODE_ACCMAG
 *        - OPERATION_MODE_ACCGYRO
 *        - OPERATION_MODE_MAGGYRO
 *        - OPERATION_MODE_AMG
 *        - OPERATION_MODE_IMUPLUS
 *        - OPERATION_MODE_COMPASS
 *        - OPERATION_MODE_M4G
 *        - OPERATION_MODE_NDOF_FMC_OFF
 *        - OPERATION_MODE_NDOF
 */
void Neorv32_BNO055::setMode(neorv32_bno055_opmode_t mode) {
    _mode = mode;  // Set the mode
    transmit(BNO055_OPR_MODE_ADDR, _mode);  // Set the sensor to the selected mode
    neorv32_cpu_delay_ms(30);  // Wait for 30 ms
}

/**
 * getMode function
 * No parameters
 * @return sensor operation mode
 */
neorv32_bno055_opmode_t Neorv32_BNO055::getMode() {
    return (neorv32_bno055_opmode_t)receive(BNO055_OPR_MODE_ADDR);  // Return the mode
}

/**
 * setAxisRemap function
 * @param remapCode
 *       remapCode values:
 *       - REMAP_CONFIG_P0
 *       - REMAP_CONFIG_P1
 *       - REMAP_CONFIG_P2
 *       - REMAP_CONFIG_P3
 *       - REMAP_CONFIG_P4
 *       - REMAP_CONFIG_P5
 *       - REMAP_CONFIG_P6
 *       - REMAP_CONFIG_P7
 *
 *       - Default remapCode is REMAP_CONFIG_P1
 */
void Neorv32_BNO055::setAxisRemap(Neorv32_BNO055::neorv32_bno055_axis_remap_config_t remapCode) {
    neorv32_bno055_opmode_t modeBack = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    transmit(BNO055_AXIS_MAP_CONFIG_ADDR, remapCode);  // Set the axis remap configuration
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms

    setMode(modeBack);  // Set the sensor to the previous mode
    neorv32_cpu_delay_ms(20);  // Wait for 20 ms
}


/**
 * setAxisSign function
 * @param remapSign
 *       remapSign values:
 *       - REMAP_SIGN_P0
 *       - REMAP_SIGN_P1
 *       - REMAP_SIGN_P2
 *       - REMAP_SIGN_P3
 *       - REMAP_SIGN_P4
 *       - REMAP_SIGN_P5
 *       - REMAP_SIGN_P6
 *       - REMAP_SIGN_P7
 *
 *       - Default remapSign is REMAP_SIGN_P1
 */
void Neorv32_BNO055::setAxisSign(Neorv32_BNO055::neorv32_bno055_axis_remap_sign_t remapSign) {
    neorv32_bno055_opmode_t modeBack = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    transmit(BNO055_AXIS_MAP_SIGN_ADDR, remapSign);  // Set the axis remap sign
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms

    setMode(modeBack);  // Set the sensor to the previous mode
    neorv32_cpu_delay_ms(20);  // Wait for 20 ms
}

/**
 * setExtCrystalUse function
 * @param usextal
 *       usextal values:
 *       - true
 *       - false
 */
void Neorv32_BNO055::setExtCrystalUse(bool usextal) {
    neorv32_bno055_opmode_t modeBack = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    uint8_t config = receive(BNO055_SYS_TRIGGER_ADDR);  // Read the SYS_TRIGGER register
    if(usextal) {
        config |= 0x80;  // Set the EXT Crystal Use bit
    } else {
        config &= 0x7F;  // Clear the EXT Crystal Use bit
    }
    transmit(BNO055_SYS_TRIGGER_ADDR, config);  // Set the SYS_TRIGGER register
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms

    setMode(modeBack);  // Set the sensor to the previous mode
    neorv32_cpu_delay_ms(20);  // Wait for 20 ms
}

/**
 * getSystemStatus function
 * @param system_status
 *        system_status information:
 *        - 0x00: Idle
 *        - 0x01: System Error
 *        - 0x02: Initializing Peripherals
 *        - 0x03: System Initialization
 *        - 0x04: Executing Self-Test
 *        - 0x05: Sensor fusion algorithm running
 *        - 0x06: System running without fusion algorithms
 * @param self_test_result
 *        self_test_result information:
 *         - 0x01: self-test passed
 *         - 0x00: self-test failed
 *            - Bit 0: Accelerometer self-test
 *            - Bit 1: Magnetometer self-test
 *            - Bit 2: Gyroscope self-test
 *            - Bit 3: MCU self-test
*            - 0x0F: all self-test passed
 * @param system_error
 *       system_error information:
 *       - 0x00: No error
 *       - 0x01: Peripheral initialization error
 *       - 0x02: System initialization error
 *       - 0x03: Self test result failed
 *       - 0x04: Register map value out of range
 *       - 0x05: Register map address out of range
 *       - 0x06: Register map write error
 *       - 0x07: BNO low power mode not available for selected operation mode
 *       - 0x08: Accelerometer power mode not available
 *       - 0x09: Fusion algorithm configuration error
 *       - 0x0A: Sensor configuration error
 *
 */
void Neorv32_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
    transmit(BNO055_PAGE_ID_ADDR, 0x00);  // Select the page 0
    if(system_status) {
        *system_status = receive(BNO055_SYS_STAT_ADDR);  // Read the SYS_STAT register
    }

    if(self_test_result) {
        *self_test_result = receive(BNO055_SELFTEST_RESULT_ADDR);  // Read the SELFTEST_RESULT register
    }

    if(system_error) {
        *system_error = receive(BNO055_SYS_ERR_ADDR);  // Read the SYS_ERR register
    }

    neorv32_cpu_delay_ms(200);  // Wait for 200 ms
}

/**
 * getRevInfo function
 * @param info
 *       info information:
 *       - accel_rev: Accelerometer revision
 *       - mag_rev: Magnetometer revision
 *       - gyro_rev: Gyroscope revision
 *       - bl_rev: Bootloader revision
 *       - sw_rev: Software revision
 *
 */
void Neorv32_BNO055::getRevInfo(Neorv32_BNO055::neorv32_bno055_rev_info_t *info) {
    uint8_t lsb, msb;

    memset(info, 0, sizeof(neorv32_bno055_rev_info_t));  // Set the info to 0
    transmit(BNO055_PAGE_ID_ADDR, 0x00);  // Select the page 0

    info->accel_rev = receive(BNO055_ACCEL_REV_ID_ADDR);  // Read the ACCEL_REV_ID register
    info->mag_rev = receive(BNO055_MAG_REV_ID_ADDR);  // Read the MAG_REV_ID register
    info->gyro_rev = receive(BNO055_GYRO_REV_ID_ADDR);  // Read the GYRO_REV_ID register
    info->bl_rev = receive(BNO055_BL_REV_ID_ADDR);  // Read the BL_REV_ID register
    lsb = receive(BNO055_SW_REV_ID_LSB_ADDR);  // Read the SW_REV_ID_LSB register
    msb = receive(BNO055_SW_REV_ID_MSB_ADDR);  // Read the SW_REV_ID_MSB register
    info->sw_rev = (((uint16_t) msb) << 8) | ((uint16_t) lsb);  // Read the SW_REV_ID_MSB register

}

/**
 * getCalibration function
 * @param system
 *        system information:
 *        - 0: not calibrated
 *        - 1: fully calibrated
 * @param gyro
 *        gyro information:
 *        - 0: not calibrated
 *        - 1: fully calibrated
 * @param accel
 *        accel information:
 *        - 0: not calibrated
 *        - 1: fully calibrated
 * @param mag
 *        mag information:
 *        - 0: not calibrated
 *        - 1: fully calibrated
 */
void Neorv32_BNO055::getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = receive(BNO055_CALIB_STAT_ADDR);  // Read the CALIB_STAT register
    if(system) {
        *system = (calData >> 6) & 0x03;  // Get the system calibration
    }
    if(gyro) {
        *gyro = (calData >> 4) & 0x03;  // Get the gyro calibration
    }
    if(accel) {
        *accel = (calData >> 2) & 0x03;  // Get the accel calibration
    }
    if(mag) {
        *mag = calData & 0x03;  // Get the mag calibration
    }
}

/**
 * getTemp function
 * @return
 *       temperature
 */
int8_t Neorv32_BNO055::getTemp() {
    int8_t temp = (int8_t)receive(BNO055_TEMP_ADDR);  // Read the TEMP register
    return temp;  // Return the temperature
}

/**
 * getVector function
 * @param vector
 *        vector_type information:
 *        - VECTOR_ACCELEROMETER
 *        - VECTOR_MAGNETOMETER
 *        - VECTOR_GYROSCOPE
 *        - VECTOR_EULER
 *        - VECTOR_LINEARACCEL
 *        - VECTOR_GRAVITY
 * @param data
 *       data information:
 *       - x: x-axis data
 *       - y: y-axis data
 *       - z: z-axis data
 *
 * @return
 *        vector
 */
 void Neorv32_BNO055::getVector(Neorv32_BNO055::neorv32_vector_type_t vector, int16_t *data = nullptr) {
    memset(data, 0, sizeof(int16_t) * 3);  // Set the data to 0

    uint8_t lsb, msb;
    switch(vector) {
        case VECTOR_ACCELEROMETER:
            lsb = receive(BNO055_ACCEL_DATA_X_LSB_ADDR);  // Read the ACCEL_DATA_X_LSB register
            msb = receive(BNO055_ACCEL_DATA_X_MSB_ADDR);  // Read the ACCEL_DATA_X_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the x-axis accelerometer data

            lsb = receive(BNO055_ACCEL_DATA_Y_LSB_ADDR);  // Read the ACCEL_DATA_Y_LSB register
            msb = receive(BNO055_ACCEL_DATA_Y_MSB_ADDR);  // Read the ACCEL_DATA_Y_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the y-axis accelerometer data

            lsb = receive(BNO055_ACCEL_DATA_Z_LSB_ADDR);  // Read the ACCEL_DATA_Z_LSB register
            msb = receive(BNO055_ACCEL_DATA_Z_MSB_ADDR);  // Read the ACCEL_DATA_Z_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the z-axis accelerometer data
            break;

        case VECTOR_MAGNETOMETER:
            lsb = receive(BNO055_MAG_DATA_X_LSB_ADDR);  // Read the MAG_DATA_X_LSB register
            msb = receive(BNO055_MAG_DATA_X_MSB_ADDR);  // Read the MAG_DATA_X_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the x-axis magnetometer data

            lsb = receive(BNO055_MAG_DATA_Y_LSB_ADDR);  // Read the MAG_DATA_Y_LSB register
            msb = receive(BNO055_MAG_DATA_Y_MSB_ADDR);  // Read the MAG_DATA_Y_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the y-axis magnetometer

            lsb = receive(BNO055_MAG_DATA_Z_LSB_ADDR);  // Read the MAG_DATA_Z_LSB register
            msb = receive(BNO055_MAG_DATA_Z_MSB_ADDR);  // Read the MAG_DATA_Z_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the z-axis magnetometer data
            break;

        case VECTOR_GYROSCOPE:
            lsb = receive(BNO055_GYRO_DATA_X_LSB_ADDR);  // Read the GYRO_DATA_X_LSB register
            msb = receive(BNO055_GYRO_DATA_X_MSB_ADDR);  // Read the GYRO_DATA_X_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the x-axis gyroscope data

            lsb = receive(BNO055_GYRO_DATA_Y_LSB_ADDR);  // Read the GYRO_DATA_Y_LSB register
            msb = receive(BNO055_GYRO_DATA_Y_MSB_ADDR);  // Read the GYRO_DATA_Y_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the y-axis gyroscope data

            lsb = receive(BNO055_GYRO_DATA_Z_LSB_ADDR);  // Read the GYRO_DATA_Z_LSB register
            msb = receive(BNO055_GYRO_DATA_Z_MSB_ADDR);  // Read the GYRO_DATA_Z_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the z-axis gyroscope data
            break;

        case VECTOR_EULER:
            lsb = receive(BNO055_EULER_H_LSB_ADDR);  // Read the EULER_H_LSB register
            msb = receive(BNO055_EULER_H_MSB_ADDR);  // Read the EULER_H_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the heading data

            lsb = receive(BNO055_EULER_R_LSB_ADDR);  // Read the EULER_R_LSB register
            msb = receive(BNO055_EULER_R_MSB_ADDR);  // Read the EULER_R_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the roll data

            lsb = receive(BNO055_EULER_P_LSB_ADDR);  // Read the EULER_P_LSB register
            msb = receive(BNO055_EULER_P_MSB_ADDR);  // Read the EULER_P_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the pitch data
            break;

        case VECTOR_LINEARACCEL:
            lsb = receive(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);  // Read the LINEAR_ACCEL_DATA_X_LSB register
            msb = receive(BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR);  // Read the LINEAR_ACCEL_DATA_X_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the x-axis linear acceleration data

            lsb = receive(BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR);  // Read the LINEAR_ACCEL_DATA_Y_LSB register
            msb = receive(BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR);  // Read the LINEAR_ACCEL_DATA_Y_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the y-axis linear acceleration data

            lsb = receive(BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR);  // Read the LINEAR_ACCEL_DATA_Z_LSB register
            msb = receive(BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR);  // Read the LINEAR_ACCEL_DATA_Z_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the z-axis linear acceleration data
            break;

        case VECTOR_GRAVITY:
            lsb = receive(BNO055_GRAVITY_DATA_X_LSB_ADDR);  // Read the GRAVITY_DATA_X_LSB register
            msb = receive(BNO055_GRAVITY_DATA_X_MSB_ADDR);  // Read the GRAVITY_DATA_X_MSB register
            data[0] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the x-axis gravity data

            lsb = receive(BNO055_GRAVITY_DATA_Y_LSB_ADDR);  // Read the GRAVITY_DATA_Y_LSB register
            msb = receive(BNO055_GRAVITY_DATA_Y_MSB_ADDR);  // Read the GRAVITY_DATA_Y_MSB register
            data[1] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the y-axis gravity data

            lsb = receive(BNO055_GRAVITY_DATA_Z_LSB_ADDR);  // Read the GRAVITY_DATA_Z_LSB register
            msb = receive(BNO055_GRAVITY_DATA_Z_MSB_ADDR);  // Read the GRAVITY_DATA_Z_MSB register
            data[2] = (((int16_t) msb) << 8) | ((int16_t) lsb);  // Get the z-axis gravity data
            break;
    }
}


/**
 * getQuat function
 * @param w
 *       w information:
 *       - w data
 * @param x
 *      x information:
 *      - x data
 * @param y
 *     y information:
 *     - y data
 * @param z
 *     z information:
 *     - z data
 */
void Neorv32_BNO055::getQuat(uint16_t *w, uint16_t *x, uint16_t *y, uint16_t *z) {
    uint8_t lsb, msb;

    lsb = receive(BNO055_QUATERNION_DATA_W_LSB_ADDR);  // Read the QUATERNION_DATA_W_LSB register
    msb = receive(BNO055_QUATERNION_DATA_W_MSB_ADDR);  // Read the QUATERNION_DATA_W_MSB register
    *w = (((uint16_t) msb) << 8) | ((uint16_t) lsb);  // Get the w data

    lsb = receive(BNO055_QUATERNION_DATA_X_LSB_ADDR);  // Read the QUATERNION_DATA_X_LSB register
    msb = receive(BNO055_QUATERNION_DATA_X_MSB_ADDR);  // Read the QUATERNION_DATA_X_MSB register
    *x = (((uint16_t) msb) << 8) | ((uint16_t) lsb);  // Get the x data

    lsb = receive(BNO055_QUATERNION_DATA_Y_LSB_ADDR);  // Read the QUATERNION_DATA_Y_LSB register
    msb = receive(BNO055_QUATERNION_DATA_Y_MSB_ADDR);  // Read the QUATERNION_DATA_Y_MSB register
    *y = (((uint16_t) msb) << 8) | ((uint16_t) lsb);  // Get the y data

    lsb = receive(BNO055_QUATERNION_DATA_Z_LSB_ADDR);  // Read the QUATERNION_DATA_Z_LSB register
    msb = receive(BNO055_QUATERNION_DATA_Z_MSB_ADDR);  // Read the QUATERNION_DATA_Z_MSB register
    *z = (((uint16_t) msb) << 8) | ((uint16_t) lsb);  // Get the z data
}

/**
 * getSensorOffsets function
 * @param calibData
 *       calibData information:
 *       - calibration data
 * @return
 *       true if the sensor is fully calibrated, false otherwise
 */
bool Neorv32_BNO055::getSensorOffsets(uint8_t *calibData) {
    if(isFullyCalibrated()) {
        neorv32_bno055_opmode_t lastMode = _mode;  // Get the current mode
        setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode

        receiveLen(BNO055_ACCEL_DATA_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);  // Read the sensor offsets
        setMode(lastMode);  // Set the sensor to the previous mode
        return true;  // Return true
    }
    return false;  // Return false
}

/**
 * getSensorOffsets function
 * @param offsets_type
 *      offsets_type information:
 *      - accel_offset_x: x-axis accelerometer offset
 *      - accel_offset_y: y-axis accelerometer offset
 *      - accel_offset_z: z-axis accelerometer offset
 *      - mag_offset_x: x-axis magnetometer offset
 *      - mag_offset_y: y-axis magnetometer offset
 *      - mag_offset_z: z-axis magnetometer offset
 *      - gyro_offset_x: x-axis gyroscope offset
 *      - gyro_offset_y: y-axis gyroscope offset
 *      - gyro_offset_z: z-axis gyroscope offset
 *      - accel_radius: accelerometer radius
 *      - mag_radius: magnetometer radius
 *
 *
 */
bool Neorv32_BNO055::getSensorOffsets(neorv32_bno055_offsets_t &offsets_type) {
    if(isFullyCalibrated()) {
        neorv32_bno055_opmode_t lastMode = _mode;  // Get the current mode
        setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
        neorv32_cpu_delay_ms(25);  // Wait for 25 ms

        offsets_type.accel_offset_x = (receive(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (receive(ACCEL_OFFSET_X_LSB_ADDR));  // Read the ACCEL_OFFSET_X_MSB and ACCEL_OFFSET_X_LSB registers
        offsets_type.accel_offset_y = (receive(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (receive(ACCEL_OFFSET_Y_LSB_ADDR));  // Read the ACCEL_OFFSET_Y_MSB and ACCEL_OFFSET_Y_LSB registers
        offsets_type.accel_offset_z = (receive(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (receive(ACCEL_OFFSET_Z_LSB_ADDR));  // Read the ACCEL_OFFSET_Z_MSB and ACCEL_OFFSET_Z_LSB registers

        offsets_type.mag_offset_x = (receive(MAG_OFFSET_X_MSB_ADDR) << 8) | (receive(MAG_OFFSET_X_LSB_ADDR));  // Read the MAG_OFFSET_X_MSB and MAG_OFFSET_X_LSB registers
        offsets_type.mag_offset_y = (receive(MAG_OFFSET_Y_MSB_ADDR) << 8) | (receive(MAG_OFFSET_Y_LSB_ADDR));  // Read the MAG_OFFSET_Y_MSB and MAG_OFFSET_Y_LSB registers
        offsets_type.mag_offset_z = (receive(MAG_OFFSET_Z_MSB_ADDR) << 8) | (receive(MAG_OFFSET_Z_LSB_ADDR));  // Read the MAG_OFFSET_Z_MSB and MAG_OFFSET_Z_LSB registers

        offsets_type.gyro_offset_x = (receive(GYRO_OFFSET_X_MSB_ADDR) << 8) | (receive(GYRO_OFFSET_X_LSB_ADDR));  // Read the GYRO_OFFSET_X_MSB and GYRO_OFFSET_X_LSB registers
        offsets_type.gyro_offset_y = (receive(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (receive(GYRO_OFFSET_Y_LSB_ADDR));  // Read the GYRO_OFFSET_Y_MSB and GYRO_OFFSET_Y_LSB registers
        offsets_type.gyro_offset_z = (receive(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (receive(GYRO_OFFSET_Z_LSB_ADDR));  // Read the GYRO_OFFSET_Z_MSB and GYRO_OFFSET_Z_LSB registers

        offsets_type.accel_radius = (receive(ACCEL_RADIUS_MSB_ADDR) << 8) | (receive(ACCEL_RADIUS_LSB_ADDR));  // Read the ACCEL_RADIUS_MSB and ACCEL_RADIUS_LSB registers
        offsets_type.mag_radius = (receive(MAG_RADIUS_MSB_ADDR) << 8) | (receive(MAG_RADIUS_LSB_ADDR));  // Read the MAG_RADIUS_MSB and MAG_RADIUS_LSB registers


        setMode(lastMode);  // Set the sensor to the previous mode
        return true;  // Return true
    }
    return false;  // Return false
 }

/**
 * setSensorOffsets function
 * @param calibData
 *      calibData information:
 *      - calibration data
 */
void Neorv32_BNO055::setSensorOffsets(const uint8_t *calibData) {
    neorv32_bno055_opmode_t lastMode = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    transmit(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);  // Set the x-axis accelerometer offset
    transmit(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);  // Set the x-axis accelerometer offset
    transmit(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);  // Set the y-axis accelerometer offset
    transmit(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);  // Set the y-axis accelerometer offset
    transmit(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);  // Set the z-axis accelerometer offset
    transmit(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);  // Set the z-axis accelerometer offset

    transmit(MAG_OFFSET_X_LSB_ADDR, calibData[6]);  // Set the x-axis magnetometer offset
    transmit(MAG_OFFSET_X_MSB_ADDR, calibData[7]);  // Set the x-axis magnetometer offset
    transmit(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);  // Set the y-axis magnetometer offset
    transmit(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);  // Set the y-axis magnetometer offset
    transmit(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);  // Set the z-axis magnetometer offset
    transmit(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);  // Set the z-axis magnetometer offset

    transmit(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);  // Set the x-axis gyroscope offset
    transmit(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);  // Set the x-axis gyroscope offset
    transmit(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);  // Set the y-axis gyroscope offset
    transmit(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);  // Set the y-axis gyroscope offset
    transmit(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);  // Set the z-axis gyroscope offset
    transmit(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);  // Set the z-axis gyroscope offset

    transmit(ACCEL_RADIUS_LSB_ADDR, calibData[18]);  // Set the accelerometer radius
    transmit(ACCEL_RADIUS_MSB_ADDR, calibData[19]);  // Set the accelerometer radius

    transmit(MAG_RADIUS_LSB_ADDR, calibData[20]);  // Set the magnetometer radius
    transmit(MAG_RADIUS_MSB_ADDR, calibData[21]);  // Set the magnetometer radius

    setMode(lastMode);  // Set the sensor to the previous mode
}

/**
 * setSensorOffsets function
 * @param offsets_type
 *     offsets_type information:
 *     - accel_offset_x: x-axis accelerometer offset
 *     - accel_offset_y: y-axis accelerometer offset
 *     - accel_offset_z: z-axis accelerometer offset
 *     - mag_offset_x: x-axis magnetometer offset
 *     - mag_offset_y: y-axis magnetometer offset
 *     - mag_offset_z: z-axis magnetometer offset
 *     - gyro_offset_x: x-axis gyroscope offset
 *     - gyro_offset_y: y-axis gyroscope offset
 *     - gyro_offset_z: z-axis gyroscope offset
 *     - accel_radius: accelerometer radius
 *     - mag_radius: magnetometer radius
 *
 */
void Neorv32_BNO055::setSensorOffsets(const neorv32_bno055_offsets_t &offsets_type) {
    neorv32_bno055_opmode_t lastMode = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    transmit(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x00FF);  // Set the x-axis accelerometer offset
    transmit(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x00FF);  // Set the x-axis accelerometer offset
    transmit(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x00FF);  // Set the y-axis accelerometer offset
    transmit(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x00FF);  // Set the y-axis accelerometer offset
    transmit(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x00FF);  // Set the z-axis accelerometer offset
    transmit(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x00FF);  // Set the z-axis accelerometer offset

    transmit(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x00FF);  // Set the x-axis magnetometer offset
    transmit(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x00FF);  // Set the x-axis magnetometer offset
    transmit(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x00FF);  // Set the y-axis magnetometer offset
    transmit(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x00FF);  // Set the y-axis magnetometer offset
    transmit(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x00FF);  // Set the z-axis magnetometer offset
    transmit(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x00FF);  // Set the z-axis magnetometer offset

    transmit(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x00FF);  // Set the x-axis gyroscope offset
    transmit(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x00FF);  // Set the x-axis gyroscope offset
    transmit(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x00FF);  // Set the y-axis gyroscope offset
    transmit(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x00FF);  // Set the y-axis gyroscope offset
    transmit(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x00FF);  // Set the z-axis gyroscope offset
    transmit(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x00FF);  // Set the z-axis gyroscope offset

    transmit(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x00FF);  // Set the accelerometer radius
    transmit(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x00FF);  // Set the accelerometer radius

    transmit(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x00FF);  // Set the magnetometer radius
    transmit(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x00FF);  // Set the magnetometer radius

    setMode(lastMode);  // Set the sensor to the previous mode

}

/**
 * isFullyCalibrated function
 * @return
 *      true if the sensor is fully calibrated, false otherwise
 *
 */
bool Neorv32_BNO055::isFullyCalibrated() {
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);  // Get the calibration data

    switch(_mode){
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        case OPERATION_MODE_AMG:
            return (accel == 3 && mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}

/**
 * enterSuspendMode function
 * No parameters
 */
void Neorv32_BNO055::enterSuspendMode() {
    neorv32_bno055_opmode_t modeBack = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms
    transmit(BNO055_PWR_MODE_ADDR, 0x02);  // Set the power mode to suspend
    setMode(modeBack);  // Set the sensor to the previous mode
    neorv32_cpu_delay_ms(20);  // Wait for 20 ms
}

/**
 * enterNormalMode function
 * No parameters
 */
void Neorv32_BNO055::enterNormalMode() {
    neorv32_bno055_opmode_t modeBack = _mode;  // Get the current mode
    setMode(OPERATION_MODE_CONFIG);  // Set the sensor to CONFIG mode
    neorv32_cpu_delay_ms(25);  // Wait for 25 ms

    transmit(BNO055_PWR_MODE_ADDR, 0x00);  // Set the power mode to normal
    setMode(modeBack);  // Set the sensor to the previous mode
    neorv32_cpu_delay_ms(20);  // Wait for 20 ms
}

/**
 * transmit function
 * @param reg
 * @param value
 * @return
 *     true if the transmission is successful, false otherwise
 */
bool Neorv32_BNO055::transmit(neorv32_bno055_reg_t reg, uint8_t value) {
    if(neorv32_uart0_available()) {
        neorv32_uart0_putc(reg);  // Send the register address
        neorv32_uart0_putc(value);  // Send the value
        return true;  // Return true
    }
    return false;  // Return false
}

/**
 * receive function
 * @param reg
 * @return
 *     the received value
 */
uint8_t Neorv32_BNO055::receive(neorv32_bno055_reg_t reg) {

    uint8_t value;
    neorv32_uart0_putc( reg);  // Send the register address
    neorv32_cpu_delay_ms(10);  // Wait for 10 ms
    value = (uint8_t)neorv32_uart0_getc();  // Get the value

    return value;  // Return the value
}

/**
 * receiveLen function
 * @param reg
 * @param data
 * @param length
 * @return
 *    true if the reception is successful, false otherwise
 *
 */
bool Neorv32_BNO055::receiveLen(neorv32_bno055_reg_t reg, uint8_t *buffer, uint8_t len) {
    if(neorv32_uart0_available()) {
        neorv32_uart0_putc( reg);  // Send the register address
        neorv32_cpu_delay_ms(10);  // Wait for 10 ms
        for(uint8_t i = 0; i < len; i++) {

            buffer[i] = (uint8_t)neorv32_uart0_getc();  // Get the value
        }
        return true;  // Return true
    }
    return false;  // Return false
}



/**
* Destructor
 * Free allocated memory
 *
*/
Neorv32_BNO055::~Neorv32_BNO055() {
    neorv32_uart0_printf("Destructor called\n");
}