#include <mbed.h>
#include <cmath>

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00010000
#define SPI_FLAG 1
#define OUT_X_L 0x28
#define SAMPLES 64
#define FFT_SIZE (SAMPLES / 2)

EventFlags flags;

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define TREMOR_THRESHOLD 0.1f
#define TREMOR_FREQ_START 3    // 3 Hz
#define TREMOR_FREQ_END 6      // 6 Hz

DigitalOut led(LED1);  // Initialize the LED

// Function to perform bit reversal on an array
void bit_reversal(float *real, float *imag, int n) {
    int j = 0;
    for (int i = 0; i < n; ++i) {
        if (i < j) {
            std::swap(real[i], real[j]);
            std::swap(imag[i], imag[j]);
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
}

// Function to perform the Cooley-Tukey FFT
void fft(float *real, float *imag, int n) {
    bit_reversal(real, imag, n);
    for (int s = 1; s <= log2(n); ++s) {
        int m = 1 << s;
        float wm_real = cos(2 * M_PI / m);
        float wm_imag = -sin(2 * M_PI / m);
        for (int k = 0; k < n; k += m) {
            float w_real = 1.0;
            float w_imag = 0.0;
            for (int j = 0; j < m / 2; ++j) {
                int t = k + j;
                int u = k + j + m / 2;
                float t_real = w_real * real[u] - w_imag * imag[u];
                float t_imag = w_real * imag[u] + w_imag * real[u];
                real[u] = real[t] - t_real;
                imag[u] = imag[t] - t_imag;
                real[t] += t_real;
                imag[t] += t_imag;
                float w_temp = w_real * wm_real - w_imag * wm_imag;
                w_imag = w_real * wm_imag + w_imag * wm_real;
                w_real = w_temp;
            }
        }
    }
}

// Function to detect tremors based on FFT output
bool detect_tremor(float *fft_output, int size, float &tremor_intensity) {
    tremor_intensity = 0.0f;
    bool tremor_detected = false;

    for (int i = TREMOR_FREQ_START; i <= TREMOR_FREQ_END; i++) {
        if (fft_output[i] > TREMOR_THRESHOLD) {
            tremor_detected = true;
            tremor_intensity += fft_output[i];
        }
    }

    if(tremor_intensity < 9.0 || tremor_intensity == false){
        return false;
    }
    // printf("TESTING INTERMEDIATE RESULTS:  Intensity = %4.5f\n", tremor_intensity);
    tremor_intensity = tremor_intensity - 9.0;
    if(tremor_intensity >= 100){
        tremor_intensity = 100;
    }

    return tremor_detected;
}

int main() {
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    int8_t write_buf[32], read_buf[32];

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

    // FFT buffers and variables
    float input_buffer[SAMPLES];
    float imag_buffer[SAMPLES];
    int sample_index = 0;

    while(1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((int16_t)read_buf[2]) << 8) | ((int16_t) read_buf[1]);
        raw_gy = (((int16_t)read_buf[4]) << 8) | ((int16_t) read_buf[3]);
        raw_gz = (((int16_t)read_buf[6]) << 8) | ((int16_t) read_buf[5]);

        // Print the raw data
        // printf("RAW data values-> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual velocity values
        printf("Velocity Values -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

        input_buffer[sample_index++] = gx;
        if (sample_index >= SAMPLES) {
            sample_index = 0;
            memset(imag_buffer, 0, sizeof(imag_buffer));

            // Perform the FFT
            fft(input_buffer, imag_buffer, SAMPLES);

            // Calculate the magnitude of the FFT output
            float fft_output[FFT_SIZE];
            for (int i = 0; i < FFT_SIZE; i++) {
                fft_output[i] = sqrtf(input_buffer[i] * input_buffer[i] + imag_buffer[i] * imag_buffer[i]);
            }

            // Print the FFT output
            printf("FFT Output:\n");
            for (int i = 0; i < FFT_SIZE; i++) {
                printf("%4.5f ", fft_output[i]);
            }
            printf("\n\n");

            // Detect tremors
            float tremor_intensity;
            if (detect_tremor(fft_output, FFT_SIZE, tremor_intensity)) {
                printf("Tremor detected!\n");
                printf("Tremor Intensity: %d\n\n", static_cast<int>(ceil(tremor_intensity)));
                // Turn on the led
                led = 1;
            } else {
                printf("No tremor detected.\n\n");
                // Turn off the led
                led = 0;
            }
        }

        thread_sleep_for(100);
    }
}
