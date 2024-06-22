#include <mbed.h>

// =================================================
// * Recitation 5: SPI and Gyroscope *
// =================================================

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot. 
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)


int main()
{
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    while(1){

        uint16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        // Print the raw values for debugging 
        printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

            printf(">x_axis: %d|g \n", raw_gx);
            printf(">y_axis: %d|g \n", raw_gy);
            printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual values
        printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

        thread_sleep_for(100);
    }

}
