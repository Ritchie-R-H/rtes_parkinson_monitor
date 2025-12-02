// src/signal_proc.h
#pragma once
#include <cstddef>

struct DetectionResult {
    float dominant_freq;   // Clock speed Hz
    float dominant_mag;    // Main frequency amplitude
    float tremor_level;    // 0-100
    float dyskinesia_level;// 0-100
    float fog_level;       // 0-100
};

class SignalProcessor {
public:
    SignalProcessor(float fs, size_t window_size);

    // Analysis of acceleration modulus and gyroscope modulus
    DetectionResult analyze(const float *acc_mag,
                            const float *gyro_mag);

private:
    float _fs;
    size_t _window_size;
};
