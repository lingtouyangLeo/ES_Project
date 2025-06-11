// stm32_fft_tremor_project/main.cpp
#include "mbed.h"
#include "arm_math.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &serial_port; }

#define FFT_SIZE 256
#define SAMPLE_RATE 100.0f

#define WHO_AM_I  0x0F // ID register
#define OUTX_L_XL 0x28 // Linear acceleration sensor X-axis low byte output register
#define OUTX_H_XL 0x29 // Linear acceleration sensor X-axis high byte output register
#define OUTY_L_XL 0x2A // Linear acceleration sensor X-axis low byte output register
#define OUTY_H_XL 0x2B // Linear acceleration sensor X-axis high byte output register
#define OUTZ_L_XL 0x2C // Linear acceleration sensor X-axis low byte output register
#define OUTZ_H_XL 0x2D // Linear acceleration sensor X-axis high byte output register
#define CTRL1_XL  0x10 // Linear acceleration sensor control register
#define CTRL2_G   0x11 // Angular rate sensor control register

#define LSM6DSL_ADDR (0x6A << 1) // 8-bit address

// I2C for LSM6DSL (PB_11 = SDA, PB_10 = SCL)
I2C i2c(PB_11, PB_10);
InterruptIn user_button(BUTTON1);
DigitalOut dyskinesiaSignal(LED1);
DigitalOut tremorSignal(LED2);
DigitalOut led(LED3);

float32_t acc_buffer[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

arm_rfft_fast_instance_f32 fft_instance;

volatile bool beginDataCollection = false;
const float ACC_SENSITIVITY = 0.016f;

void button_pressed()
{
    beginDataCollection = true;
}

void init_LSM6DSL()
{
    char data[2];

    // set CTRL1_XL register: ODR_XL = 0100 (104 Hz), FS_XL = 00 (±2g) 
    data[0] = CTRL1_XL;
    data[1] = 0x40; // 0100 0000
    i2c.write(LSM6DSL_ADDR, data, 2);

    // Disable gyroscope
    data[0] = CTRL2_G;
    data[1] = 0x00;
    i2c.write(LSM6DSL_ADDR, data, 2);
}

uint8_t read_reg8(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
}

int16_t read_reg16(uint8_t low_reg, uint8_t high_reg)
{
    char buf[2] = {0};
    buf[0] = read_reg8(low_reg);
    buf[1] = read_reg8(high_reg);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

void read_accel_data()
{
    for (int i = 0; i < FFT_SIZE; i++)
    {
        int16_t x = read_reg16(OUTX_L_XL, OUTX_H_XL);   // Read X value
        int16_t y = read_reg16(OUTY_L_XL, OUTY_H_XL);   // Read Y value
        int16_t z = read_reg16(OUTZ_L_XL, OUTZ_H_XL);   // Read Z value

        printf("X=%d\t\tY=%d\t\tZ=%d\r\n", x, y, z);

        acc_buffer[i] = sqrtf(x * x + y * y + z * z);   // Calculate vector sum
        ThisThread::sleep_for(10ms);                    // Sample every 10 ms for 100 Hz
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

void turnOnLeds()
{
    led = 1;
    tremorSignal = 1;
    dyskinesiaSignal = 1;
}

void turnOffLeds()
{
    led = 0;
    tremorSignal = 0;
    dyskinesiaSignal = 0;
}

void flashLightsBasedOnIntensityOfMovement(int intensity)
{
    // Add 2 to the intensity so that light will blink at least once even at lower frequency in range
    for (int i = 0; i < (intensity + 2); i++) 
    {
        led = !led; // Will flash based on intensity with 10 levels of intensity
        ThisThread::sleep_for(250ms);
    }
    led = 1; // Keep on Led to signal result available
}

void detect_condition()
{
    float resolution = SAMPLE_RATE / FFT_SIZE;
    float32_t max_val;
    uint32_t max_index;
    arm_max_f32(magnitude, FFT_SIZE / 2, &max_val, &max_index);

    float freq = resolution * max_index;
    int freq_int = (int)freq;
    int freq_frac = (int)((freq - freq_int) * 100);
    int mag_int = (int)max_val;
    int mag_frac = (int)((max_val - mag_int) * 100);

    printf("Peak frequency: %d.%02d Hz\r\n", freq_int, freq_frac);
    printf("Peak magnitude: %d.%02d\r\n", mag_int, mag_frac);

    if (freq >= 3.0f && freq <= 5.0f)
    {
        printf("Tremor detected\r\n");
        tremorSignal = 1; // Turn on single LED to signal tremor
        dyskinesiaSignal = 0;

        int intensity = (freq - 3.0f) * 10; // Values of (freq - 3) * 10 will exist from 0 - 20
        flashLightsBasedOnIntensityOfMovement(intensity);
    }
    else if (freq > 5.0f && freq <= 7.0f)
    {
        printf("Dyskinesia detected\r\n");
        dyskinesiaSignal = 1; // Turn on single LED to signal dyskinesia
        tremorSignal = 0;

        int intensity = (freq - 5.0f) * 10; // Values of (freq - 5) * 10 will exist from 0 - 20
        flashLightsBasedOnIntensityOfMovement(intensity);
    }
    else
    {
        printf("No abnormal movement\r\n");
        turnOnLeds();   // If all lights on and no blinking light then no abnormal movement
                        // LED3 signifies result is available, LED1 & LED2 on signify no abnormality
                        // We chose to turn on both lights instead of turning both off because users
                        // tend to like to see an active response like a light on instead of lights off
        ThisThread::sleep_for(1000ms);
    }
}

int main()
{
    // Setup I2C at 400kHz
    i2c.frequency(400000);

    // Initial LED blink to indicate system start
    turnOnLeds();
    ThisThread::sleep_for(500ms);
    turnOffLeds();
    led = 1; // Set led to 1, when this LED is off this is an indication the device is recording data

    printf("Shake, Rattle, and Roll - STM32 Project\r\n");

    // Initialize FFT
    arm_status status = arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    if (status != ARM_MATH_SUCCESS) {
        printf("FFT initialization failed\r\n");
        while(1);
    }

    // Configure sensors and verify connection
    init_LSM6DSL();
    uint8_t id = read_reg8(WHO_AM_I);
    if (id != 0x6A) {
        printf("WHO_AM_I = 0x%02X (Expected: 0x6A)\r\n", id);
        printf("Error: LSM6DSL sensor not found!\r\n");
        while (1);
    }

    // Set up interrupt routine when button is pressed to set beginDataCollection flag to true
    user_button.fall(&button_pressed);

    printf("Ready for data collection. Press button to begin sampling.\r\n");

    while (true)
    {
        if (beginDataCollection)            // When button pressed begin data collection
        {
            turnOffLeds();                  // Turn LEDs off to clear previous results and signal data is being collected
            beginDataCollection = false;    // Reset button pressed flag
            printf("Sampling...\r\n");
            read_accel_data();              // Collect data from sensors
            led = 1;                        // Turn LED on when sampling is completed
            printf("FFT analysis...\r\n");
            run_fft();                      // Run FFT analysis
            detect_condition();             // Detect condition and set appropriate LEDS
                                            // LED results will stay set until new measurement is taken
                                            // LED3/LED4 will blink based on intensity of observed condition and then
                                            // remain on, if no abnormal motion is detected LED3 will just remain on
                                            // LED1 signals tremors, LED2 signals dyskinesia, if both LEDs are on
                                            // no abnormal motion was detected
        }
        ThisThread::sleep_for(1000ms);      // Hold result for 1 second before next measurement
    }
}