//
// Created by ekon-ihc on 9/10/24.
//

#ifndef NEORV32_PROJECT_NEORV32_BNO055_H
#define NEORV32_PROJECT_NEORV32_BNO055_H

#include <string.h>
#include <neorv32.h>
#include <neorv32_uart.h>
#include <neorv32_legacy.h>

/** UART0 is used for communication with BNO055 */

/** BNO055 ADDRESS */
#define BNO055_ADDRESS 0x28

/** BNO055 ID */
#define BNO055_ID 0xA0

/** BNO055 ID ADDRESS */
#define BNO055_ID_ADDR 0x00


/** BNO055 NUMBER OF OFFSET REGISTERS */
#define NUM_BNO055_OFFSET_REGISTERS (22) // Number of offset registers

/** ************************************************************************* */
/** NEORV32 UART*/
#define BAUD_RATE 115200  // Baud rate (the communication of BNO055 with uart requires 115200 baud rate)
/** ************************************************************************* */

#define uint8_t unsigned char

/** BNO055 sensors offsets */
typedef struct {
    int16_t accel_offset_x; /**< x acceleration offset */
    int16_t accel_offset_y; /**< y acceleration offset */
    int16_t accel_offset_z; /**< z acceleration offset */

    int16_t mag_offset_x; /**< x magnetometer offset */
    int16_t mag_offset_y; /**< y magnetometer offset */
    int16_t mag_offset_z; /**< z magnetometer offset */

    int16_t gyro_offset_x; /**< x gyroscrope offset */
    int16_t gyro_offset_y; /**< y gyroscrope offset */
    int16_t gyro_offset_z; /**< z gyroscrope offset */

    int16_t accel_radius; /**< acceleration radius */

    int16_t mag_radius; /**< magnetometer radius */
} neorv32_bno055_offsets_t;

/** BNO055 operation modes */
typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} neorv32_bno055_opmode_t;

/** BNO055 Registers */
typedef enum {
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_EULER_R_LSB_ADDR = 0X1C,
    BNO055_EULER_R_MSB_ADDR = 0X1D,
    BNO055_EULER_P_LSB_ADDR = 0X1E,
    BNO055_EULER_P_MSB_ADDR = 0X1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
} neorv32_bno055_reg_t;

class Neorv32_BNO055 {

private:
    int32_t _sensorID; /**< sensor ID */
    neorv32_bno055_opmode_t _mode; /**< mode */

    // neorv32_bno055_opmode_t _mode; /**< mode */

    bool transmit(neorv32_bno055_reg_t reg, uint8_t value); // Transmit 8
    uint8_t receive(neorv32_bno055_reg_t reg); // Receive 8
    bool receiveLen(neorv32_bno055_reg_t reg, uint8_t *buffer, uint8_t len); // Receive length


public:



    /** BNO055 power settings */
    typedef enum {
        POWER_MODE_NORMAL = 0X00,
        POWER_MODE_LOWPOWER = 0X01,
        POWER_MODE_SUSPEND = 0X02
    } neorv32_bno055_powermode_t;

    /** BNO055 axis remap settings */
    typedef enum {
        REMAP_CONFIG_P0 = 0x21,
        REMAP_CONFIG_P1 = 0x24, // default
        REMAP_CONFIG_P2 = 0x24,
        REMAP_CONFIG_P3 = 0x21,
        REMAP_CONFIG_P4 = 0x24,
        REMAP_CONFIG_P5 = 0x21,
        REMAP_CONFIG_P6 = 0x21,
        REMAP_CONFIG_P7 = 0x24
    } neorv32_bno055_axis_remap_config_t;

    /** BNO055 axis remap signs */
    typedef enum {
        REMAP_SIGN_P0 = 0x04,
        REMAP_SIGN_P1 = 0x00, // default
        REMAP_SIGN_P2 = 0x06,
        REMAP_SIGN_P3 = 0x02,
        REMAP_SIGN_P4 = 0x03,
        REMAP_SIGN_P5 = 0x01,
        REMAP_SIGN_P6 = 0x07,
        REMAP_SIGN_P7 = 0x05
    } neorv32_bno055_axis_remap_sign_t;

    /** BNO055 axis remap orientations */
    typedef enum {
        REMAP_X = 0x00,
        REMAP_Y = 0x01,
        REMAP_Z = 0x02
    } neorv32_bno055_axis_remap_orient_t;

    /** BNO055 calibration status */
    typedef enum {
        CALIB_STAT_ACC = 0x01,
        CALIB_STAT_MAG = 0x02,
        CALIB_STAT_GYR = 0x04,
        CALIB_STAT_SYS = 0x08
    } neorv32_bno055_calib_stat_t;

    /** BNO055 system status */
    typedef enum {
        SYS_STATUS_IDLE = 0x00,
        SYS_STATUS_ERROR = 0x01,
        SYS_STATUS_INITIALIZING_PERIPHERALS = 0x02,
        SYS_STATUS_SYSTEM_INITIALIZATION = 0x03,
        SYS_STATUS_EXECUTING_SELFTEST = 0x04,
        SYS_STATUS_SENSOR_FUSION_ALGORITHM_RUNNING = 0x05,
        SYS_STATUS_SYSTEM_RUNNING_WITHOUT_FUSION_ALGORITHM = 0x06
    } neorv32_bno055_sys_stat_t;

    /** BNO055 system error */
    typedef enum {
        SYS_ERROR_NONE = 0x00,
        SYS_ERROR_PERIPHERAL_INITIALIZATION_ERROR = 0x01,
        SYS_ERROR_SYSTEM_INITIALIZATION_ERROR = 0x02,
        SYS_ERROR_SELFTEST_RESULT_FAILED = 0x03,
        SYS_ERROR_REGMAP_INVALID_ADDRESS = 0x04,
        SYS_ERROR_REGMAP_WRITE_ERROR = 0x05,
        SYS_ERROR_LOWPOWER_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE = 0x06,
        SYS_ERROR_ACC_PWR_MODE_NOT_AVAILABLE = 0x07,
        SYS_ERROR_FUSION_ALGO_CONFIGURATION_ERROR = 0x08,
        SYS_ERROR_SENSOR_CONFIG_ERROR = 0x09
    } neorv32_bno055_sys_err_t;

    /** BNO055 system trigger */
    typedef enum {
        SYS_TRIGGER_RESET = 0x20,
        SYS_TRIGGER_SELF_TEST = 0x01,
        SYS_TRIGGER_FUSION_MODE = 0x80
    } neorv32_bno055_sys_trigger_t;

    /** BNO055 temperature sources */
    typedef enum {
        TEMP_SOURCE_ACC = 0x00,
        TEMP_SOURCE_GYR = 0x01
    } neorv32_bno055_temp_source_t;

    /** BNO055 unit selection */
    typedef enum {
        UNIT_SEL_AND_CEl_DPS_RAD_MS2 = 0b10000100,   // Andriod / Celsius / Dps / Rads / m/s^2
        UNIT_SEL_AND_CEl_RPS_RAD_MS2 = 0b10000110,   // Andriod / Celsius / Rps / Rads / m/s^2
        UNIT_SEL_AND_CEl_DPS_DEG_MS2 = 0b10000000,   // Andriod / Celsius / Dps / Deg / m/s^2
        UNIT_SEL_AND_CEl_RPS_DEG_MS2 = 0b10000010,   // Andriod / Celsius / Rps / Deg / m/s^2
        UNIT_SEL_AND_F_DPS_RAD_MS2 = 0b10010100,     // Andriod / Fahrenheit / Dps / Rads / m/s^2
        UNIT_SEL_AND_F_RPS_RAD_MS2 = 0b10010110,     // Andriod / Fahrenheit / Rps / Rads / m/s^2
        UNIT_SEL_AND_F_DPS_DEG_MS2 = 0b10010000,     // Andriod / Fahrenheit / Dps / Deg / m/s^2
        UNIT_SEL_AND_F_RPS_DEG_MS2 = 0b10010010,     // Andriod / Fahrenheit / Rps / Deg / m/s^2

        UNIT_SEL_WIN_CEl_DPS_RAD_MS2 = 0b00000100,   // Windows / Celsius / Dps / Rads / m/s^2
        UNIT_SEL_WIN_CEl_RPS_RAD_MS2 = 0b00000110,   // Windows / Celsius / Rps / Rads / m/s^2
        UNIT_SEL_WIN_CEl_DPS_DEG_MS2 = 0b00000000,   // Windows / Celsius / Dps / Deg / m/s^2
        UNIT_SEL_WIN_CEl_RPS_DEG_MS2 = 0b00000010,   // Windows / Celsius / Rps / Deg / m/s^2
        UNIT_SEL_WIN_F_DPS_RAD_MS2 = 0b00010100,     // Windows / Fahrenheit / Dps / Rads / m/s^2
        UNIT_SEL_WIN_F_RPS_RAD_MS2 = 0b00010110,     // Windows / Fahrenheit / Rps / Rads / m/s^2
        UNIT_SEL_WIN_F_DPS_DEG_MS2 = 0b00010000,     // Windows / Fahrenheit / Dps / Deg / m/s^2
        UNIT_SEL_WIN_F_RPS_DEG_MS2 = 0b00010010,     // Windows / Fahrenheit / Rps / Deg / m/s^2

    } neorv32_bno055_unit_sel_t;

    /** structure to represent revisions **/
    typedef struct {
        uint8_t accel_rev; /**< acceleration rev */
        uint8_t mag_rev;   /**< magnetometer rev */
        uint8_t gyro_rev;  /**< gyroscrope rev */
        uint16_t sw_rev;   /**< SW rev */
        uint8_t bl_rev;    /**< bootloader rev */
    } neorv32_bno055_rev_info_t;

    /** Vector Mappings **/
    typedef enum {
        VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
        VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
        VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
        VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
        VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
        VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
    } neorv32_vector_type_t;

    /** BNO055 constructor */
    Neorv32_BNO055(int32_t sensorID = -1); // Constructor

    bool begin(neorv32_bno055_opmode_t mode, neorv32_bno055_unit_sel_t unitSel ); // begin

    void setMode(neorv32_bno055_opmode_t mode); // Set mode
    neorv32_bno055_opmode_t getMode(); // Get mode
    void setAxisRemap(neorv32_bno055_axis_remap_config_t remapCode ); // Set axis remap
    void setAxisSign(neorv32_bno055_axis_remap_sign_t remapSign); // Set axis sign
    void setExtCrystalUse(bool usextal); // Set external crystal use
    void getRevInfo(neorv32_bno055_rev_info_t *info); // Get revision info
    void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error); // Get system status
    void getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag); // Get calibration
    int8_t getTemp(); // Get temperature
    void getVector(neorv32_vector_type_t vector, int16_t *data); // Get vector
    void getQuat(uint16_t *w, uint16_t *x, uint16_t *y, uint16_t *z); // Get quaternion
    bool getSensorOffsets(uint8_t *calibData); // Get sensor offsets
    bool getSensorOffsets(neorv32_bno055_offsets_t &offsets_type); // Get sensor offsets
    void setSensorOffsets(const uint8_t *calibData); // Set sensor offsets
    void setSensorOffsets(const neorv32_bno055_offsets_t &offsets_type); // Set sensor offsets
    bool isFullyCalibrated(); // Is fully calibrated

    /* Power managments functions */
    void enterSuspendMode();
    void enterNormalMode();












    /** BNO055 destructor */
    ~Neorv32_BNO055(); // Destructor


};


#endif //NEORV32_PROJECT_NEORV32_BNO055_H
