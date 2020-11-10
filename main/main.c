/* I2C SSD1309 test
*/
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "include/wire.h"
#include "ssd1306.h"
#include "delay.h"
#include "bno055.h"

#define ACK_VAL 0x00
#define NACK_VAL 0x01

#define CONFIG_FREERTOS_HZ 100

//Accel BNO055
#define I2C_PORT I2C_NUMBER_1
#define USE_EXTERNAL_CRYSTAL 1

//SD card
static const char *TAG = "SD test:";
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

//PSU - Power Supply Unit
#define PSU_I2C_ADDR 0x58
#define PSU_I2C_REG_PWR_CR 0x01
#define PSU_I2C_REG_PWR_CR_HV_EN 0x03
#define PSU_I2C_REG_RTC_TR1 0x06
#define PSU_I2C_REG_RTC_TR2 0x07
#define PSU_I2C_REG_RTC_TR3 0x08
#define PSU_I2C_REG_RTC_DR1 0x09
#define PSU_I2C_REG_RTC_DR2 0x0A
#define PSU_I2C_REG_RTC_DR3 0x0B
uint8_t rtc_tr1_val = 0;

//Mic ADC
#define NO_OF_SAMPLES   1    //Multisampling

//Pulse sensor MAX30102
#define MAX30102_I2C_ADDR 0x57

//Temp. sensor TMP101
#define TMP101_I2C_ADDR 0x4a
uint8_t temp[4];
char str_buf[10];

//Display reset
#define DISP_RESET 21

//Button
#define BTN_PIN 27

//Delay micros
#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}
void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

void app_main(void)
{
    /*//Mic, ADC test
    //Set MIC_GAIN pin to LOW
    gpio_pad_select_gpio(26);
    gpio_set_direction(26, GPIO_MODE_OUTPUT);
    gpio_set_level(26, 0);
    delay(250);
    //Check if Two Point or Vref are burned into eFuse
    check_efuse(); 

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);

    //Continuously sample ADC1
    int max_adc_reading = 0;
    int min_adc_reading = 4095;
    int it_num = 0;
        while (1) {
        int adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw(ADC1_CHANNEL_3);
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        //uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //printf("Raw: %d\t\n", adc_reading);
        if (adc_reading > max_adc_reading) {
            max_adc_reading = adc_reading;
        }
        if (adc_reading < min_adc_reading) {
            min_adc_reading = adc_reading;
        }
        it_num++;
        if (it_num == 800) {
            it_num = 0;
            printf("min_val=%d\t  max_val=%d\n", min_adc_reading, max_adc_reading);
            min_adc_reading = 4095;
            max_adc_reading = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));


    }*/
    
    //Accel BNO055 test
    /*bno055_config_t bno_conf;

    esp_err_t err;
    err = bno055_set_default_conf( &bno_conf);
    bno_conf.i2c_address = BNO055_ADDRESS_A;
    bno_conf.sda_io_num = 26;
    bno_conf.scl_io_num = 25;
    err = bno055_open(I2C_PORT, &bno_conf);
    printf("bno055_open() returned 0x%02X \n", err);

    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        err = bno055_close(I2C_PORT);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }

    err = bno055_set_ext_crystal_use(I2C_PORT, USE_EXTERNAL_CRYSTAL);
    if( err != ESP_OK ) {
        printf("Couldn't set external crystal use\n");
        err = bno055_close(I2C_PORT);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);

    err = bno055_set_opmode(I2C_PORT, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);

    bno055_quaternion_t quat;
    bno055_vec3_t lin_accel, grav;
    uint8_t calib;
    int n_mes = 0;
    while (1)
    {
        err = bno055_get_fusion_data(I2C_PORT, &quat, &lin_accel, &grav);
        if( err != ESP_OK ) {
            printf("bno055_get_fusion() returned error: %02x \n", err);
            exit(2);
        }
        err = bno055_get_calib_status_byte(I2C_PORT, &calib);
        if( err != ESP_OK ) {
            printf("bno055_get_calib_status_byte() returned error: %02x \n", err);
            exit(2);
        }
        printf( "%d"
                "#calib_status,%d"
                "#linacc,%5f,%5f,%5f"
                "#rotvec,%.5f,%.5f,%.5f,%.5f"
                "#grav,%.5f,%.5f,%.5f\n",
                n_mes,
                calib,
                lin_accel.x, lin_accel.y, lin_accel.z,
                quat.w, quat.x, quat.y, quat.z,
                grav.x, grav.y, grav.z);
        n_mes++;

        delay(10);
    }*/

    printf("ESP32 sensors test\n");
        
    //I2C init
    /*ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C master inited OK\n");
    delay(550);*/

    //Pulse oximeter sensor MAX30102 test
    /*{
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);    
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("OK: MAX30102 found on I2C address %d\n", MAX30102_I2C_ADDR);
        } else {
            printf("ERROR: MAX30102 not found\n");
        }
    }*/

    //PSU controller RTC test. I2C addr: 0x58, RTC registers 0x06:0x1A
    /*{
        //i2c_read_register(I2C_NUM_1, PSU_I2C_ADDR...)
        //read test ID register 0x1C
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, 0x1C, 1);
        
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_READ, 1);
        i2c_master_read_byte(cmd, &rtc_tr1_val, NACK_VAL);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 5000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("PSU I2C ID register read OK!\n");
        } else {
            printf("PSU I2C ID register read error!\n");
            ESP_ERROR_CHECK(ret);
        }
        printf("RTC_TR1: %d\n", rtc_tr1_val);

        //read test RTC_TR1 register
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, PSU_I2C_REG_RTC_TR1, 1);
        
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_READ, 1);
        i2c_master_read_byte(cmd, &rtc_tr1_val, NACK_VAL);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("PSU I2C RTC_TR1 register read OK!\n");
        } else {
            printf("PSU I2C RTC_TR1 register read error!\n");
        }
        printf("RTC_TR1: %d\n", rtc_tr1_val);

        //write test
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, PSU_I2C_REG_RTC_TR1, 1);
        i2c_master_write_byte(cmd, 0b00010000, 1);
        i2c_master_stop(cmd);    
        ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("PSU I2C RTC_TR1 register write OK!\n");
        } else {
            printf("PSU I2C RTC_TR1 register write error!\n");
        }

    }*/

    delay(550);

    /*
    //Temperature sensor TMP101 (addr 0x4a)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TMP101_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, 0x01, 1);
        i2c_master_write_byte(cmd, 0x60, 1);
        i2c_master_stop(cmd);    
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("OK: TMP101 found on I2C address %d\n", TMP101_I2C_ADDR);
        } else {
            printf("ERROR: TMP101 not found\n");
        }

        if (ret == ESP_OK) {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (TMP101_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
            i2c_master_write_byte(cmd, 0x00, 1);
            i2c_master_stop(cmd);    
            ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);

            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (TMP101_I2C_ADDR << 1) | I2C_MASTER_READ, 1);
            i2c_master_read(cmd, temp, 2, ACK_VAL);
            i2c_master_read_byte(cmd, temp + 2, NACK_VAL);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("Temperature read OK: %.2f\n", (temp[0] * 256 + (temp[1] & 0xF0)) / 16 * 0.0625);
                sprintf(str_buf, "%.2fC",(temp[0] * 256 + (temp[1] & 0xF0)) / 16 * 0.0625);
            }
        }
    }

    delay(550);
    
    //PSU controller I2C addr: 0x58, reg: 0x01, HV_EN (bit 3): set to 1 (12V ON)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PSU_I2C_ADDR << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, PSU_I2C_REG_PWR_CR, 1);
        i2c_master_write_byte(cmd, 0b00001011, 1);
        i2c_master_stop(cmd);    
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("PSU I2C write PWR_CR register OK!\n");
        } else {
            printf("PSU I2C write error!\n");
        }
    }

    delay(550);

    //Set display reset pin to HIGH (normal operation)
    gpio_pad_select_gpio(DISP_RESET);
    gpio_set_direction(DISP_RESET, GPIO_MODE_OUTPUT);
    //gpio_set_level(DISP_RESET, 0);
    //delay(250);
    gpio_set_level(DISP_RESET, 1);
    delay(250);

    ESP_ERROR_CHECK(i2c_device_ping(SSD1306_I2C_ADDR));
    printf("SSD1309 display found on I2C addr %d\n", SSD1306_I2C_ADDR);
    
    //OLED init
    ssd1306_Init();
    ssd1306_SetCursor(47, 27);
    //ssd1306_WriteString("Hello", Font_7x10, White);
    ssd1306_WriteString(str_buf, Font_7x10, White);
    //ssd1306_Fill(White);
    ssd1306_UpdateScreen();
    delay(1000);
    //ssd1306_Fill(Black);

    //Button test
    ssd1306_Fill(Black);
    ssd1306_SetCursor(1, 20);
    ssd1306_WriteString("Press button", Font_7x10, White);
    ssd1306_UpdateScreen();
    for (size_t i = 0; i < 120; i++)
    {
        if (gpio_get_level(BTN_PIN) == 0) {
            ssd1306_SetCursor(20, 40);
            ssd1306_WriteString("pressed", Font_7x10, White);
            ssd1306_UpdateScreen();
            printf("Button pressed\n");
            delay(2000);
            break;
        }
        printf("Button not pressed, pin state: %d\n", gpio_get_level(BTN_PIN));
        delay(25);
    }
    */
    
    //Beep test
    /*ESP_LOGI("BEEP TEST:", "short beep signal");
    gpio_pad_select_gpio(9);
    gpio_set_direction(9, GPIO_MODE_OUTPUT);
    for (size_t i = 0; i < 1000; i++)
    {
        gpio_set_level(9, 1);
        delayMicroseconds(500);
        gpio_set_level(9, 0);
        delayMicroseconds(500);
    }*/
    
    
    
    /*//SD card test
    //Set PWR_SD pin to HIGH (EN)
    //gpio_pad_select_gpio(25);
    //gpio_set_direction(25, GPIO_MODE_OUTPUT);
    //gpio_set_level(25, 1);
    //delay(250);

    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/sdcard/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello %s!\n", card->cid.name);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    // Open file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/sdcard/hello.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);
    
    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGI(TAG, "Card unmounted");*/
    


    //restart ESP32
    for (int i = 3; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        delay(2500);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
