// =============================================================================
//  Challenge 02 — Frequency Estimator
// =============================================================================
//
//  Virtual hardware:
//    ADC Ch 0  →  io.analog_read(0)      Process sensor signal (0–4095)
//    OUT reg 3 →  io.write_reg(3, …)     Frequency estimate in centiHz
//                                        e.g. write_reg(3, 4733) = 47.33 Hz
//
//  Goal:
//    Measure the frequency of the signal on ADC channel 0 and publish your
//    estimate continuously via register 3.
//
//  Read README.md before starting.
// =============================================================================

// === How it works ===
// Signal Flow: Signal from adc -> DC Removal -> Phase Detector -> PI Controller -> Oscillator -> Frequency Estimate -> Mooving Average Filter -> Output Display
// 1. Signal from ADC (raw sensor input)
// 2. DC Removal: estimates and subtracts DC to get AC component (ac_signal)
// 3. Phase Detector: multiplies AC signal by local oscillator cosine
// 4. PI Controller: adjusts oscillator frequency based on phase error
// 5. Oscillator: integrates frequency to produce phase theta
// 6. Frequency Estimate: omega / 2pi converted to cHz
// 7. Moving Average Filter: smooths frequency over N samples
// 8. Output Display: writes filtered frequency to output register

// PLL + PI Controller: adjusts the oscillator to minimize the difference between the input AC signal 
// and the internal reference, locking the phase and frequency.

#include <trac_fw_io.hpp>
#include <cstdint>
#include <cmath>

#define PI 3.14159265f
#define INITIAL_FREQ_HZ 7.5f // Initial guess for the frequency (midpoint of 6–9 Hz)

struct MovingAverage {
	static constexpr int N = 500; // set to 500 for a 500ms window at 1 kHz sampling (satisfies the 1 sec requirement)

    uint32_t buffer[N] = { 0 };
    uint32_t sum = 0;
    int index = 0;
    bool filled = false;

    uint32_t update(uint32_t new_value) {
        sum -= buffer[index];
        buffer[index] = new_value;
        sum += new_value;

        index++;
        if (index >= N) {
            index = 0;
            filled = true;
        }

		int divisor = filled ? N : index; // Before filling the buffer, divide by the number of samples added so far
        if (divisor == 0) return new_value;

        return sum / divisor;
    }
};

int main() {
    trac_fw_io_t io;

	const float Ts = 0.001f; // 1 ms sample period (1000 Hz sampling rate)
    uint32_t last_time_calc = 0;
    uint32_t last_time_print = 0;

    // PLL
    float theta = 0.0f;
    float omega = 2.0f * PI * INITIAL_FREQ_HZ;

    float integrator = 0.0f;

	const float Kp = 10.0f; // Proportional gain of the PI controller (tuning parameter)
	const float Ki = 40.0f; // Integral gain of the PI controller (tuning parameter)

	int constexpr sample_period_ms = 1;  
	int constexpr print_period_ms = 100; // changeable print period (100 ms looks good)

    // DC removal
	float dc = 2048.0f; // midpoint of ADC range
    const float alpha_dc = 0.001f;
    
    uint32_t freq_cHz = 0;
    uint32_t freq_cHz_filtered = 0;

    // Mooving average filter
    MovingAverage ma;

    while (true) {
        uint32_t now = io.millis();

        if (now - last_time_calc >= sample_period_ms) {
            last_time_calc = now;

            float raw = io.analog_read(0);

            // Remove DC
			dc = (1.0f - alpha_dc) * dc + alpha_dc * raw; // IIR low-pass filter for DC estimation
            float ac_signal = (raw - dc) / 2048.0f;

			// Local oscillator
            float sin_t = sinf(theta);
            float cos_t = cosf(theta);

			// Phase detector
            float error = ac_signal * cos_t;

			// PI controller
            integrator += Ki * error * Ts;
            omega = (2.0f * PI * INITIAL_FREQ_HZ) + Kp * error + integrator;

			// integrates phase
            theta += omega * Ts;

            if (theta > 2.0f * PI)
                theta -= 2.0f * PI;
            
            float freq = omega / (2.0f * PI);

            freq_cHz = (uint32_t)(freq * 100.0f);
            freq_cHz_filtered = ma.update(freq_cHz);

        }

        if (now - last_time_print >= print_period_ms)
        {
			last_time_print = now;  
            io.write_reg(3, freq_cHz_filtered);
        }
    }
}
