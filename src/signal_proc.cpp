#include "signal_proc.h"
#include <cmath>
#include <algorithm>

SignalProcessor::SignalProcessor(float fs, size_t window_size)
    : _fs(fs), _window_size(window_size) {}

DetectionResult SignalProcessor::analyze(const float *acc_mag,
                                         const float *gyro_mag)
{
    DetectionResult res{};
    const size_t N = _window_size;

    // 1. Simple statistics: used for FOG determination
    float acc_mean = 0.0f, gyro_mean = 0.0f;
    for (size_t i = 0; i < N; ++i) {
        acc_mean += acc_mag[i];
        gyro_mean += gyro_mag[i];
    }
    acc_mean /= N;
    gyro_mean /= N;

    float acc_var = 0.0f, gyro_var = 0.0f;
    for (size_t i = 0; i < N; ++i) {
        float da = acc_mag[i] - acc_mean;
        float dg = gyro_mag[i] - gyro_mean;
        acc_var += da * da;
        gyro_var += dg * dg;
    }
    acc_var /= N;
    gyro_var /= N;
    float acc_std = std::sqrt(acc_var);
    float gyro_std = std::sqrt(gyro_var);

    // 2. DFT: Only calculates frequencies in the range of 0~10Hz.
    // k corresponds to the frequency f_k = k * fs / N
    float max_mag_overall = 0.0f;
    float max_freq_overall = 0.0f;

    float max_tremor_mag = 0.0f, tremor_freq = 0.0f;
    float max_dysk_mag   = 0.0f, dysk_freq   = 0.0f;

    float fs = _fs;

    // TODO: If meaningful energies are found to be above 10Hz, the value of 10.0f can be increased; 
    // otherwise, it can be reduced to decrease the computational load.
    // The maximum k corresponding to 10 Hz
    size_t k_max = static_cast<size_t>(10.0f * N / fs);
    if (k_max > N/2) k_max = N/2;

    for (size_t k = 1; k <= k_max; ++k) {
        float freq = (float)k * fs / (float)N;

        float real = 0.0f;
        float imag = 0.0f;
        for (size_t n = 0; n < N; ++n) {
            float phase = -2.0f * M_PI * k * n / (float)N;
            float x = acc_mag[n];
            real += x * std::cos(phase);
            imag += x * std::sin(phase);
        }
        float mag = std::sqrt(real * real + imag * imag);

        if (mag > max_mag_overall) {
            max_mag_overall = mag;
            max_freq_overall = freq;
        }
        
        // TODO: The frequency bands are not exactly the same for different individuals; 
        // these two intervals can be reset based on the experiment.
        // Tremor: 3-5 Hz
        if (freq >= 3.0f && freq <= 5.0f && mag > max_tremor_mag) {
            max_tremor_mag = mag;
            tremor_freq = freq;
        }
        // Dyskinesia: 5-7 Hz
        if (freq > 5.0f && freq <= 7.0f && mag > max_dysk_mag) {
            max_dysk_mag = mag;
            dysk_freq = freq;
        }
    }

    res.dominant_freq = max_freq_overall;
    res.dominant_mag  = max_mag_overall;

    // 3. Normalize to 0-100, which can be adjusted according to the actual experiment.
    const float TREMOR_MAG_REF = 50.0f;   // TODO: tune reference magnitude for tremor normalization
    const float DYSK_MAG_REF   = 50.0f;   // TODO: tune reference magnitude for dyskinesia normalization

    res.tremor_level = std::min(100.0f, (max_tremor_mag / TREMOR_MAG_REF) * 100.0f);
    res.dyskinesia_level = std::min(100.0f, (max_dysk_mag / DYSK_MAG_REF) * 100.0f);

    // 4. FOG: If std is very small, it is considered frozen.
    // TODO: tune FOG thresholds based on real gait vs freeze patterns
    const float ACC_STD_FOG_THRESH  = 0.03f; // g
    const float GYRO_STD_FOG_THRESH = 3.0f;  // dps
    if (acc_std < ACC_STD_FOG_THRESH && gyro_std < GYRO_STD_FOG_THRESH) {
        // TODO: Here's a baseline value; it's recommended to implement the actual 
        // FOG state as a state machine in the main function.
        res.fog_level = 80.0f;
    } else {
        res.fog_level = 0.0f;
    }

    return res;
}
