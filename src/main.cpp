#include "mbed.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "events/mbed_events.h"

#include "imu_sensor.h"
#include "signal_proc.h"
#include "ble_service.h"
#include <cmath>

// -------- Serial port redirection: Make printf use the USB virtual serial port --------
UnbufferedSerial pc(USBTX, USBRX, 115200);

namespace mbed {
FileHandle *mbed_override_console(int)
{
    // All printf / puts / scanf will go through this serial port
    return &pc;
}
}

using namespace std::chrono;

// -------- Global objects & parameters --------

// I2C: PB11 (SDA), PB10 (SCL)
ImuSensor imu(PB_11, PB_10);

// TODO: Tune Sampling Rate and Window Size
// Sampling parameters: 50 Hz, 3-second window
static constexpr float   SAMPLE_RATE_HZ   = 52.0f;  // 52Hz
static constexpr auto    SAMPLE_PERIOD    = 19ms;
static constexpr float   WINDOW_SECONDS   = 3.0f;
static constexpr size_t  WINDOW_SAMPLES   = (size_t)(SAMPLE_RATE_HZ * WINDOW_SECONDS);

// Buffers
float g_acc_mag[WINDOW_SAMPLES];
float g_gyro_mag[WINDOW_SAMPLES];
size_t g_sample_idx = 0;
bool   g_window_ready = false;

// LED indicator
DigitalOut led(LED1);
DigitalOut led2(LED2);

// Signal processor (internally does FFT + tremor/dyskinesia/FOG analysis)
SignalProcessor g_signal_proc(SAMPLE_RATE_HZ, WINDOW_SAMPLES);

// BLE
static events::EventQueue event_queue(16 * EVENTS_EVENT_SIZE);
BLE &ble_interface = BLE::Instance();
ParkinsonBleService *g_parkinson_service = nullptr;

// -------- Helper functions --------

static float vector_norm3(float x, float y, float z)
{
    return std::sqrt(x * x + y * y + z * z);
}

// -------- IMU sampling & window update (runs in EventQueue context) --------

void sample_imu()
{
    float ax, ay, az;
    float gx, gy, gz;

    if (!imu.read_all(ax, ay, az, gx, gy, gz)) {
        return;
    }

    float acc_mag  = vector_norm3(ax, ay, az);   // acceleration magnitude in g
    float gyro_mag = vector_norm3(gx, gy, gz);   // angular rate magnitude in dps

    g_acc_mag[g_sample_idx]  = acc_mag;
    g_gyro_mag[g_sample_idx] = gyro_mag;

    // Convert to integers: mg / dps for debug printing
    int ax_mg    = (int)(ax * 1000.0f);
    int ay_mg    = (int)(ay * 1000.0f);
    int az_mg    = (int)(az * 1000.0f);
    int amag_mg  = (int)(acc_mag * 1000.0f);
    int gmag_dps = (int)(gyro_mag);

    // Print one line approximately every 1 second (every 50 samples)
    if (g_sample_idx % 50 == 0) {
        printf("raw: ax=%dmg ay=%dmg az=%dmg |a|=%dmg |g|=%ddps\r\n",
               ax_mg, ay_mg, az_mg, amag_mg, gmag_dps);
    }

    g_sample_idx++;
    if (g_sample_idx >= WINDOW_SAMPLES) {
        g_sample_idx = 0;
        g_window_ready = true;   // One 3-second window is filled
    }

    // LED toggle indicates sampling activity
    // led = !led;
}

// -------- Window analysis + BLE update (runs in EventQueue context) --------

void process_window()
{
    if (!g_window_ready) {
        return;
    }
    g_window_ready = false;

    DetectionResult res = g_signal_proc.analyze(g_acc_mag, g_gyro_mag);

    // To avoid using %f, convert to integer representation
    int f_dom_centi = (int)(res.dominant_freq * 100.0f);   // Hz * 100

    int tremor_lvl = (int)(res.tremor_level);
    if (tremor_lvl < 0) tremor_lvl = 0;
    if (tremor_lvl > 100) tremor_lvl = 100;

    int dysk_lvl = (int)(res.dyskinesia_level);
    if (dysk_lvl < 0) dysk_lvl = 0;
    if (dysk_lvl > 100) dysk_lvl = 100;

    int fog_lvl = (int)(res.fog_level);
    if (fog_lvl < 0) fog_lvl = 0;
    if (fog_lvl > 100) fog_lvl = 100;

    printf("analysis: f_dom=%d(centiHz) tremor=%d dysk=%d fog=%d\r\n",
           f_dom_centi, tremor_lvl, dysk_lvl, fog_lvl);
    
    // Visualize the curves using teleplot
    printf(">tremor:%d\n", tremor_lvl);
    printf(">dysk:%d\n", dysk_lvl);
    printf(">fog:%d\n", fog_lvl);
    printf(">fdom:%0.2f\n", res.dominant_freq);

    const int TH_FOG = 40;
    
    if (fog_lvl >= TH_FOG) {  // FOG -> both LEDs off 
        led = 0;
        led2 = 0;
    } else if (tremor_lvl >= dysk_lvl) {  // Tremor -> turn on LED2
        led = 0;
        led2 = 1;
    } else if (dysk_lvl > tremor_lvl) {  // Dysk -> turn on LED1
        led = 1;
        led2 = 0;
    } else {
        led = 0;
        led2 = 0;
    }

    // Update BLE characteristics (3 independent characteristics)
    if (g_parkinson_service) {
        g_parkinson_service->update_levels(
            (uint8_t)tremor_lvl,
            (uint8_t)dysk_lvl,
            (uint8_t)fog_lvl
        );
    }
}

// -------- BLE-related functions --------

void start_advertising()
{
    Gap &gap = ble_interface.gap();

    ble::AdvertisingParameters adv_params(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(100)) // TODO: tune advertising interval (power vs discovery latency)
    );

    uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv_data_builder(adv_buffer);

    adv_data_builder.setFlags();
    adv_data_builder.setName("RTES_Parkinson");

    gap.setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_params);
    gap.setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE,
                              adv_data_builder.getAdvertisingData());
    gap.startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    printf("BLE advertising started.\r\n");
}

void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params)
{
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE init failed (err=%d).\r\n", params->error);
        return;
    }

    printf("BLE init done.\r\n");

    // Create the custom Parkinson service (implemented in ble_service.cpp / .h)
    g_parkinson_service = new ParkinsonBleService(ble_interface);

    start_advertising();
}

// When the BLE stack has events to process, schedule them on the EventQueue
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(mbed::callback(&context->ble, &BLE::processEvents));
}

void schedule_ble_init()
{
    ble_interface.onEventsToProcess(schedule_ble_events);
    ble_interface.init(on_ble_init_complete);
}

// -------- main --------

int main()
{
    printf("Stage D: IMU + SignalProcessor + BLE starting...\r\n");

    // Initialize IMU
    if (!imu.init()) {
        printf("IMU init failed! Halt.\r\n");
        while (true) {
            led = !led;
            ThisThread::sleep_for(200ms);
        }
    } else {
        printf("IMU init OK.\r\n");
    }

    // Initialize BLE (events are processed in the EventQueue)
    schedule_ble_init();

    // Periodically sample IMU in the EventQueue every 20 ms
    event_queue.call_every(SAMPLE_PERIOD, sample_imu);

    // Periodically check and process a complete window
    // Not strictly locked to 3 s; just check g_window_ready every 100 ms
    event_queue.call_every(100ms, process_window);

    // Enter the event loop (main thread runs IMU + BLE here)
    event_queue.dispatch_forever();
}
