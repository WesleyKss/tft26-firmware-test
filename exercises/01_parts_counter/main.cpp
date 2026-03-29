// =============================================================================
//  Exercise 01 — Parts Counter
// =============================================================================
//
//  Virtual hardware:
//    SW 0        →  io.digital_read(0)        Inductive sensor input
//    Display     →  io.write_reg(6, …)        LCD debug (see README for format)
//                   io.write_reg(7, …)
//
//  Goal:
//    Count every part that passes the sensor and show the total on the display.
//
//  Read README.md before starting.
// =============================================================================

#include <trac_fw_io.hpp>
#include <cstdint>
#include <cstring>
#include <cstdio>

// === How it works ===
// This code counts parts passing a sensor. It samples the sensor every 5 ms.
// A part is detected only if the sensor reads HIGH for 5 consecutive samples.
// After detection, the code waits for 4 consecutive LOW readings before allowing
// the next part to be counted. The count is displayed on an 8-character display

// function to write the part count to the display registers
static void write_display(trac_fw_io_t& io, uint32_t value)
{
    char buffer[9] = {};
    std::snprintf(buffer, sizeof(buffer), "%8lu", value);

    uint32_t reg_low = 0;
    uint32_t reg_high = 0;

    std::memcpy(&reg_low, buffer + 0, 4);
    std::memcpy(&reg_high, buffer + 4, 4);

    io.write_reg(6, reg_low);
    io.write_reg(7, reg_high);
}

int main() {
    trac_fw_io_t io;

    write_display(io, 0); // initialize display with zero count 

    uint32_t part_count = 0;

    io.set_pullup(0, true); // enable pull-up on input port 0
   
    const uint8_t stable_high_required = 5; // consecutive high readings to confirm part detection
    const uint8_t stable_low_required = 4;  // consecutive low readings to reset detection

    uint8_t consecutive_high = 0;
    uint8_t consecutive_low = 0;

    bool ready_to_detect = true;

    // sampling loop parameters
    uint32_t last_sample_time = 0;
	const uint32_t sample_interval_ms = 5; // fixed sampling interval value

    while (true) {
        uint32_t current_time = io.millis();

		// makes sure we sample the sensor at the defined interval
        if (current_time - last_sample_time >= sample_interval_ms) {
            last_sample_time = current_time;

            bool sensor_level = io.digital_read(0);

            if (sensor_level) {
                consecutive_high++;
                consecutive_low = 0; 
            }
            else {
                consecutive_low++;
                consecutive_high = 0;
            }

            // detect part when stable high is observed
            if (ready_to_detect && consecutive_high >= stable_high_required) {
                part_count++;
                write_display(io, part_count);
                ready_to_detect = false; // wait for reset before detecting next part
            }

            // reset detection when stable low is observed
            if (!ready_to_detect && consecutive_low >= stable_low_required) {
                ready_to_detect = true;
            }
        }
    }
}

