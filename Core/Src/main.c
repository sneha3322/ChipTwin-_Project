#include "main.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;

#define BMP280_ADDR (0x76 << 1) // BMP280 I2C address (shifted for HAL)

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
uint8_t bmp280_read_reg(uint8_t reg);
void bmp280_read_data();

// Calibration coefficients (these values should be retrieved from the sensor, but hardcoded here for simplicity)
int32_t dig_T1 = 27504; // Example calibration coefficients for BMP280
int32_t dig_T2 = 26435;
int32_t dig_T3 = -1000;

int32_t dig_P1 = 36477;
int32_t dig_P2 = -10685;
int32_t dig_P3 = 3024;
int32_t dig_P4 = 2855;
int32_t dig_P5 = 140;
int32_t dig_P6 = -7;
int32_t dig_P7 = 15500;
int32_t dig_P8 = -14600;
int32_t dig_P9 = 6000;

// Function to read a BMP280 register
uint8_t bmp280_read_reg(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR, &value, 1, HAL_MAX_DELAY);
    return value;
}

// Function to calculate temperature in Celsius
float bmp280_calculate_temperature(int32_t adc_temp) {
    int32_t var1 = ((((adc_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_temp >> 4) - ((int32_t)dig_T1)) * ((adc_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    float temperature = (t_fine * 5 + 128) >> 8;
    return temperature / 100.0; // Temperature in Celsius
}

// Function to calculate pressure in hPa
float bmp280_calculate_pressure(int32_t adc_press, int32_t t_fine) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // Avoid division by zero
    }

    int64_t p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
    return (float)p / 256.0; // Pressure in hPa
}

// Read BMP280 temperature and pressure
void bmp280_read_data() {
    uint8_t press_msb = bmp280_read_reg(0xF7);
    uint8_t press_lsb = bmp280_read_reg(0xF8);
    uint8_t press_xlsb = bmp280_read_reg(0xF9);

    uint8_t temp_msb = bmp280_read_reg(0xFA);
    uint8_t temp_lsb = bmp280_read_reg(0xFB);
    uint8_t temp_xlsb = bmp280_read_reg(0xFC);

    int32_t adc_press = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    int32_t adc_temp = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

    // Calculate temperature in Celsius
    float temperature = bmp280_calculate_temperature(adc_temp);
    // Calculate pressure in hPa
    float pressure = bmp280_calculate_pressure(adc_press, 0); // Pass t_fine (if used) for better accuracy

    // Print the temperature and pressure in human-readable units (float)
    printf("Temp: %.2f°C, Pressure: %.2f hPa\n", temperature, pressure);
    fflush(stdout);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    while (1) {
        bmp280_read_data();
        HAL_Delay(1000); // Delay of 1 second
    }
}

// System Clock Configuration
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1); // Error handler
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while (1); // Error handler
    }
}

// GPIO Initialization
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

// I2C1 Initialization
static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while (1); // Error handler
    }
}

// Override _write() function for semihosting (redirect printf to Renode)
int _write(int file, char *ptr, int len) {
    asm volatile("mov r0, #0"); // SYS_WRITE0
    asm volatile("mov r1, %0" : : "r"(ptr));
    asm volatile("bkpt #0xAB");
    return len;
}
