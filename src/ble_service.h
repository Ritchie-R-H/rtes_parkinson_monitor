#pragma once
#include "mbed.h"
#include "ble/BLE.h"
#include "ble/GattServer.h"

class ParkinsonBleService {
public:

    static const UUID SERVICE_UUID;

    ParkinsonBleService(BLE &ble);

    void update_levels(uint8_t tremor, uint8_t dyskinesia, uint8_t fog);

private:
    BLE &_ble;

    // 三个 characteristic
    uint8_t _tremor_level;
    uint8_t _dysk_level;
    uint8_t _fog_level;

    GattCharacteristic _tremor_char;
    GattCharacteristic _dysk_char;
    GattCharacteristic _fog_char;
};
