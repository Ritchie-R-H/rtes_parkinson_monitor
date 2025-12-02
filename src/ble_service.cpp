#include "ble_service.h"

static const uint8_t SERVICE_UUID_BYTES[] = {
    0x12,0x34,0x56,0x78,0x90,0xAB,0xCD,0xEF,
    0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF
};
const UUID ParkinsonBleService::SERVICE_UUID(SERVICE_UUID_BYTES);

static const uint8_t TREMOR_UUID_BYTES[] = {
    0x12,0x34,0x56,0x78,0x90,0xAB,0xCD,0xEF,
    0x01,0x23,0x45,0x67,0x00,0x00,0x00,0x01
};
static const uint8_t DYSK_UUID_BYTES[] = {
    0x12,0x34,0x56,0x78,0x90,0xAB,0xCD,0xEF,
    0x01,0x23,0x45,0x67,0x00,0x00,0x00,0x02
};
static const uint8_t FOG_UUID_BYTES[] = {
    0x12,0x34,0x56,0x78,0x90,0xAB,0xCD,0xEF,
    0x01,0x23,0x45,0x67,0x00,0x00,0x00,0x03
};

ParkinsonBleService::ParkinsonBleService(BLE &ble)
    : _ble(ble),
      _tremor_level(0),
      _dysk_level(0),
      _fog_level(0),
      _tremor_char(UUID(TREMOR_UUID_BYTES),
                   &_tremor_level,
                   sizeof(_tremor_level),
                   sizeof(_tremor_level),
                   GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                   GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      _dysk_char(UUID(DYSK_UUID_BYTES),
                 &_dysk_level,
                 sizeof(_dysk_level),
                 sizeof(_dysk_level),
                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),
      _fog_char(UUID(FOG_UUID_BYTES),
                &_fog_level,
                sizeof(_fog_level),
                sizeof(_fog_level),
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
{
    GattCharacteristic *chars[] = {
        &_tremor_char,
        &_dysk_char,
        &_fog_char
    };

    GattService service(SERVICE_UUID, chars,
                        sizeof(chars) / sizeof(chars[0]));

    ble.gattServer().addService(service);
}

void ParkinsonBleService::update_levels(uint8_t tremor,
                                        uint8_t dyskinesia,
                                        uint8_t fog)
{
    _tremor_level = tremor;
    _dysk_level   = dyskinesia;
    _fog_level    = fog;

    _ble.gattServer().write(_tremor_char.getValueHandle(),
                            &_tremor_level, sizeof(_tremor_level));
    _ble.gattServer().write(_dysk_char.getValueHandle(),
                            &_dysk_level, sizeof(_dysk_level));
    _ble.gattServer().write(_fog_char.getValueHandle(),
                            &_fog_level, sizeof(_fog_level));
}
