#pragma once
#include "mbed.h"

class ImuSensor {
public:
    ImuSensor(PinName sda, PinName scl);

    bool init();  // Configure ODR, measuring range, etc.

    // Read acceleration (unit: g)
    bool read_accel(float &ax, float &ay, float &az);

    // Read gyroscope readings (unit: dps)
    bool read_gyro(float &gx, float &gy, float &gz);

    // Read both and return whether it was successful.
    bool read_all(float &ax, float &ay, float &az,
                  float &gx, float &gy, float &gz);

private:
    I2C _i2c;
    int _addr;

    bool write_reg(uint8_t reg, uint8_t value);
    bool read_regs(uint8_t start_reg, uint8_t *data, uint8_t len);

    float _acc_sensitivity;   // mg/LSB
    float _gyro_sensitivity;  // mdps/LSB
};
