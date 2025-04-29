// stm32_fft_tremor_project/main.cpp
#include "mbed.h"
#include "arm_math.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &serial_port; }

#define FFT_SIZE 256
#define SAMPLE_RATE 100.0f

// I2C for LSM6DSL (PB_11 = SDA, PB_10 = SCL)
I2C i2c(PB_11, PB_10);
#define LSM6DSL_ADDR (0x6A << 1) // 8-bit address

float32_t acc_buffer[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

arm_rfft_fast_instance_f32 fft_instance;

InterruptIn user_button(BUTTON1);
DigitalOut led(PB_14);

volatile bool start_flag = false;

void button_pressed()
{
    start_flag = true;
}

void init_LSM6DSL()
{
    char data[2];

    // CTRL1_XL = 0x10: ODR_XL = 104 Hz, ±2g
    data[0] = 0x10;
    data[1] = 0x40; // 0100 0000 = 104 Hz, 2g, BW = 400Hz
    i2c.write(LSM6DSL_ADDR, data, 2);

    // CTRL2_G = 0x11: disable gyroscope
    data[0] = 0x11;
    data[1] = 0x00;
    i2c.write(LSM6DSL_ADDR, data, 2);
}

int16_t read_reg16(uint8_t low_reg)
{
    char reg = low_reg;
    char buf[2] = {0};
    i2c.write(LSM6DSL_ADDR, &reg, 1, true);
    i2c.read(LSM6DSL_ADDR, buf, 2);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

void read_accel_data()
{
    for (int i = 0; i < FFT_SIZE; i++)
    {
        int16_t x = read_reg16(0x28); // OUTX_L_XL
        int16_t y = read_reg16(0x2A); // OUTY_L_XL
        int16_t z = read_reg16(0x2C); // OUTZ_L_XL

        printf("X=%d\tY=%d\tZ=%d\r\n", x, y, z);

        acc_buffer[i] = sqrtf(x * x + y * y + z * z);
        ThisThread::sleep_for(10ms); // 100 Hz
    }

    // Remove DC offset
    float32_t mean;
    arm_mean_f32(acc_buffer, FFT_SIZE, &mean);
    for (int i = 0; i < FFT_SIZE; i++)
    {
        acc_buffer[i] -= mean;
    }
}

void run_fft()
{
    arm_rfft_fast_f32(&fft_instance, acc_buffer, fft_output, 0);
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);
}

void detect_condition()
{
    float32_t max_val;
    uint32_t max_index;
    arm_max_f32(magnitude, FFT_SIZE / 2, &max_val, &max_index);

    float freq = (SAMPLE_RATE / FFT_SIZE) * max_index;
    int freq_int = (int)freq;
    int freq_frac = (int)((freq - freq_int) * 100);
    int mag_int = (int)max_val;
    int mag_frac = (int)((max_val - mag_int) * 100);

    printf("Peak frequency: %d.%02d Hz\r\n", freq_int, freq_frac);
    printf("Peak magnitude: %d.%02d\r\n", mag_int, mag_frac);

    if (freq >= 3.0f && freq <= 5.0f)
    {
        printf("Tremor detected\r\n");
        for (int i = 0; i < 5; i++)
        {
            led = !led;
            ThisThread::sleep_for(400ms);
        }
    }
    else if (freq > 5.0f && freq <= 7.0f)
    {
        printf("Dyskinesia detected\r\n");
        for (int i = 0; i < 10; i++)
        {
            led = !led;
            ThisThread::sleep_for(150ms);
        }
    }
    else
    {
        printf("No abnormal movement\r\n");
        led = 1;
        ThisThread::sleep_for(1000ms);
        led = 0;
    }
}

int main()
{
    // Initial LED blink to indicate system start
    led = 1;
    ThisThread::sleep_for(500ms);
    led = 0;

    printf("Shake, Rattle, and Roll - STM32 Project\r\n");
    user_button.fall(&button_pressed);
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    init_LSM6DSL();

    while (true)
    {
        if (start_flag)
        {
            start_flag = false;
            printf("Sampling...\r\n");
            led = 1;
            read_accel_data();
            led = 0;
            printf("FFT analysis...\r\n");
            run_fft();
            detect_condition();
        }
        ThisThread::sleep_for(100ms);
    }
}