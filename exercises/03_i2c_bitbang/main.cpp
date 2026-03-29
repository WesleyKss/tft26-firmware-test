// =============================================================================
//  Exercise 03 — I2C Sensors (Bit-bang)
// =============================================================================
//
//  Virtual hardware:
//    P8 (SCL)  →  io.digital_write(8, …) / io.digital_read(8)
//    P9 (SDA)  →  io.digital_write(9, …) / io.digital_read(9)
//
//  PART 1 — TMP64 temperature sensor at I2C address 0x48
//    Register 0x0F  WHO_AM_I   — 1 byte  (expected: 0xA5)
//    Register 0x00  TEMP_RAW   — 4 bytes, big-endian int32_t, milli-Celsius
//
//  PART 2 — Unknown humidity sensor (same register layout, address unknown)
//    Register 0x0F  WHO_AM_I   — 1 byte
//    Register 0x00  HUM_RAW    — 4 bytes, big-endian int32_t, milli-percent
//
//  Goal (Part 1):
//    1. Implement an I2C master via bit-bang on P8/P9.
//    2. Read WHO_AM_I from TMP64 and confirm the sensor is present.
//    3. Read TEMP_RAW in a loop and print the temperature in °C every second.
//    4. Update display registers 6–7 with the formatted temperature string.
//
//  Goal (Part 2):
//    5. Scan the I2C bus (addresses 0x08–0x77) and print every responding address.
//    6. For each unknown device found, read its WHO_AM_I and print it.
//    7. Add the humidity sensor to the 1 Hz loop: read HUM_RAW and print %RH.
//
//  Read README.md before starting.
// =============================================================================

#include <trac_fw_io.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>

// === How it works ===
// SoftI2C (bit-bang) — software I2C master
// start()/stop() control SDA/SCL, write_byte() sends byte + reads ACK
// read_byte() reads byte from sensor, ack = continue reading
// read_reg(): writes address + register, then reads big-endian data
// check_device(): sends address, returns true if sensor responds
// be_to_int32(): converts 4 big-endian bytes to int32_t
// 1 Hz loop reads TMP64 & humidity sensor, prints values, updates display

//  SoftI2C (bit-bang) — Open-drain implementation
class SoftI2C {

private:
    trac_fw_io_t& io;
    uint8_t SCL;
    uint8_t SDA;

public:
    SoftI2C(trac_fw_io_t& io_, uint8_t scl, uint8_t sda)
        : io(io_), SCL(scl), SDA(sda) {
    }

    void init() {
        io.set_pullup(SCL, true);
        io.set_pullup(SDA, true);
        release_scl();
        release_sda();
    }

    // -- low-level primitives (open-drain) --
    
    void release_scl() { 
        io.digital_write(SCL, 1); 
    }
    void pull_scl() { 
        io.digital_write(SCL, 0); 
    }

    void release_sda() { 
        io.digital_write(SDA, 1); 
    }
    void pull_sda() { 
        io.digital_write(SDA, 0); 
    }

    bool read_sda() { 
        return io.digital_read(SDA); 
    }

    void delay() {
        for (volatile int i = 0; i < 40; i++);
    }

    // -- I2C protocol --
    void start() { 
        // start condition: SDA goes LOW while SCL is HIGH
        release_sda();
        release_scl();
        delay();

		pull_sda();   
        delay();

        pull_scl();
    }

    void stop() {
        // stop condition: SDA goes HIGH while SCL is HIGH
        pull_sda();
        release_scl();
        delay();

		release_sda(); 
        delay();
    }

    bool write_byte(uint8_t data) {
		// MSB first
        for (int i = 0; i < 8; i++) {
            if (data & 0x80) release_sda();
            else             pull_sda();

			// Create a clock pulse
            release_scl();
            delay();
            pull_scl();

            data <<= 1;
        }

        // ACK
        release_sda();
        release_scl();
        delay();

        bool ack = !read_sda(); // ACK = 0

        pull_scl();
        return ack;
    }

    uint8_t read_byte(bool ack) {
        uint8_t data = 0;

        // Let the slave drive SDA for 8 bits
        release_sda();

        for (int i = 0; i < 8; i++) {
            data <<= 1;

            release_scl();
            delay();

            if (read_sda())
                data |= 1;

            pull_scl();
        }

		// Writes ACK/NACK bit to the slave
        if (ack) pull_sda();
        else     release_sda();

		// generates clock pulse for ACK/NACK
        release_scl();
        delay();
        pull_scl();

        release_sda();

        return data;
    }

    // -- High-level helpers --
    bool read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, int len) {
        start();

		// Write slave address + write bit, then register address
        if (!write_byte(addr << 1)) return false;
        if (!write_byte(reg))       return false;

        start(); // repeated start

		// Write slave address + read bit,
        if (!write_byte((addr << 1) | 1)) return false;

        // then read data
        for (int i = 0; i < len; i++) {
            buf[i] = read_byte(i < (len - 1));
        }

        stop();
        return true;
    }

    bool check_device(uint8_t addr) {
        start();
        bool ack = write_byte(addr << 1);
        stop();
        return ack;
    }


}; // end SoftI2C class

// -- helpers --
// convert 4 bytes in big-endian order to int32_t
int32_t be_to_int32(uint8_t* b) {
    return (int32_t(b[0]) << 24) |
        (int32_t(b[1]) << 16) |
        (int32_t(b[2]) << 8) |
        (int32_t(b[3]));
}

// -- functions to write formatted temperature and humidity to display registers --
void write_temp_display(trac_fw_io_t& io, float temp) {
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%8.3f", temp);

    uint32_t r6, r7;
    std::memcpy(&r6, buf + 0, 4);
    std::memcpy(&r7, buf + 4, 4);

    io.write_reg(6, r6);
    io.write_reg(7, r7);
}

void write_hum_display(trac_fw_io_t& io, float hum) {
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%7.3f%%", hum);

    uint32_t r4, r5;
    std::memcpy(&r4, buf + 0, 4);
    std::memcpy(&r5, buf + 4, 4);

    io.write_reg(4, r4);
    io.write_reg(5, r5);
}
// ------------

int main() {
    trac_fw_io_t io;

	// SCL = P8, SDA = P9 
    SoftI2C i2c(io, 8, 9);
    i2c.init();

    // -- TMP64 check --
    uint8_t who = 0;
    if (i2c.read_reg(0x48, 0x0F, &who, 1)) {
        printf("TMP64 WHO_AM_I = 0x%02X\n\n", who);
    }

    // -- bus scan --
    uint8_t hum_addr = 0;

	// HMD10 Datasheet, range 0x08–0x77
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c.check_device(addr)) {
            printf("Device found at 0x%02X\n", addr);

            uint8_t id;
            if (i2c.read_reg(addr, 0x0F, &id, 1)) {
                printf("  WHO_AM_I = 0x%02X\n\n", id);

                if (addr != 0x48) {
					hum_addr = addr; // In this particular case, we can assume its the humidity sensor
                }
            }
        }
    }

    // -- loop 1 Hz --
    uint32_t last = 0;

    while (true) {
        uint32_t now = io.millis();

        if (now - last >= 1000) {
            last = now;

            // -- Temperature --
            uint8_t tbuf[4];
            if (i2c.read_reg(0x48, 0x00, tbuf, 4)) {
                int32_t raw = be_to_int32(tbuf);
                float temp = raw / 1000.0f;

                printf("Temp (Celsius): %.3f\n", temp);
                write_temp_display(io, temp);
            }

            // -- Humidity --
            if (hum_addr != 0) {
                uint8_t hbuf[4];
                if (i2c.read_reg(hum_addr, 0x00, hbuf, 4)) {
                    int32_t raw = be_to_int32(hbuf);
                    float hum = raw / 1000.0f;

                    printf("Hum (%%): %.3f\n\n", hum);
                    write_hum_display(io, hum);
                }
            }
        }
    }
}