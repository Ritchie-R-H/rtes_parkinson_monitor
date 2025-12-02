#include "imu_sensor.h"

#define LSM6DSL_ADDR      (0x6A << 1)  // 8-bit address format
#define CTRL1_XL          0x10
#define CTRL2_G           0x11
#define OUTX_L_G          0x22
#define OUTX_L_XL         0x28

ImuSensor::ImuSensor(PinName sda, PinName scl)
    : _i2c(sda, scl), _addr(LSM6DSL_ADDR),
      _acc_sensitivity(0.061f),   // Default ±2g, 0.061 mg/LSB:contentReference[oaicite:9]{index=9}
      _gyro_sensitivity(8.75f)    // Default ±250 dps, 8.75 mdps/LSB:contentReference[oaicite:10]{index=10}
{
    _i2c.frequency(400000); // 400kHz I2C
}

bool ImuSensor::write_reg(uint8_t reg, uint8_t value)
{
    char buf[2] = { (char)reg, (char)value };
    int ret = _i2c.write(_addr, buf, 2);
    return (ret == 0);
}

bool ImuSensor::read_regs(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    char reg = (char)start_reg;
    if (_i2c.write(_addr, &reg, 1, true) != 0) {
        return false;
    }
    if (_i2c.read(_addr, (char *)data, len) != 0) {
        return false;
    }
    return true;
}

bool ImuSensor::init()
{
    // Configuration acceleration: ODR = 52Hz, FS = ±2g
    // datasheet: CTRL1_XL (0x10)
    // ODR_XL[3:0] bits: 0100 = 52 Hz; FS_XL[1:0] = 00 = ±2g:contentReference[oaicite:11]{index=11}
    // Which is 0b0100 0000 = 0x40
    if (!write_reg(CTRL1_XL, 0x40)) {
        return false;
    }
    _acc_sensitivity = 0.061f; // mg/LSB

    // Configure gyroscope: ODR = 52Hz, FS = ±250 dps
    // CTRL2_G (0x11), ODR_G[3:0] = 0100 => 52 Hz; FS_G[1:0] = 00 => ±250 dps
    // => 0b0100 0000 = 0x40
    if (!write_reg(CTRL2_G, 0x40)) {
        return false;
    }
    _gyro_sensitivity = 8.75f; // mdps/LSB

    return true;
}

bool ImuSensor::read_accel(float &ax, float &ay, float &az)
{
    uint8_t data[6];
    if (!read_regs(OUTX_L_XL, data, 6)) {
        return false;
    }
    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // raw -> mg
    ax = raw_x * _acc_sensitivity / 1000.0f;
    ay = raw_y * _acc_sensitivity / 1000.0f;
    az = raw_z * _acc_sensitivity / 1000.0f;
    return true;
}

bool ImuSensor::read_gyro(float &gx, float &gy, float &gz)
{
    uint8_t data[6];
    if (!read_regs(OUTX_L_G, data, 6)) {
        return false;
    }
    int16_t raw_x = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_y = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_z = (int16_t)(data[5] << 8 | data[4]);

    // raw -> dps
    gx = raw_x * _gyro_sensitivity / 1000.0f;
    gy = raw_y * _gyro_sensitivity / 1000.0f;
    gz = raw_z * _gyro_sensitivity / 1000.0f;
    return true;
}

bool ImuSensor::read_all(float &ax, float &ay, float &az,
                         float &gx, float &gy, float &gz)
{
    bool ok1 = read_accel(ax, ay, az);
    bool ok2 = read_gyro(gx, gy, gz);
    return ok1 && ok2;
}
