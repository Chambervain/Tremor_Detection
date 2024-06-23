#include <mbed.h>
#include "arm_math.h"

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

#define FILTER_SIZE 10  // Number of samples for the moving average filter
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define ALPHA 0.1  // Smoothing factor for EMA

EventFlags flags;

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// Function to compute the moving average
float compute_moving_average(float *buffer, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

int main() {
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

    // Buffers for moving average filter
    float y_filter_buffer[FILTER_SIZE] = {0}, z_filter_buffer[FILTER_SIZE] = {0};
    int filter_index = 0;
    float ema_gx = 0.0f;

    while(1) {
        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        uint16_t raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        uint16_t raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        uint16_t raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);
        float gx = ((float) raw_gx) * SCALING_FACTOR;
        float gy = ((float) raw_gy) * SCALING_FACTOR;
        float gz = ((float) raw_gz) * SCALING_FACTOR;

        // Update the EMA for the x-axis
        ema_gx = ALPHA * gx + (1 - ALPHA) * ema_gx;

        // Update the filter buffer for y and z axes
        y_filter_buffer[filter_index] = gy;
        z_filter_buffer[filter_index] = gz;
        filter_index = (filter_index + 1) % FILTER_SIZE;

        // Compute the smoothed values using the moving average filter for y and z axes
        float smooth_gy = compute_moving_average(y_filter_buffer, FILTER_SIZE);
        float smooth_gz = compute_moving_average(z_filter_buffer, FILTER_SIZE);

        // Print the smoothed values
        printf("Smoothed -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", (-1)*ema_gx+11, (-1)*smooth_gy+20, (-1)*smooth_gz+20);

        thread_sleep_for(100);
    }
}
