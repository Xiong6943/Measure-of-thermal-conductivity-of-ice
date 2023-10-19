#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"
#include <driver/dac.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/sdspi_host.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stdlib.h>
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "ads111x.h"

#define SDA_GPIO 21
#define SCL_GPIO 22
#define ADDRESS ADS111X_ADDR_GND // connect ADDR pin to GND
#define GAIN ADS111X_GAIN_4V096

void app_main(void)
{
    i2c_dev_t dev;

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while (ads111x_init_desc(&dev, ADDRESS, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    ads111x_set_mode(&dev, ADS111X_MODE_CONTUNOUS);
    ads111x_set_data_rate(&dev, ADS111X_DATA_RATE_128);

    ads111x_set_input_mux(&dev, ADS111X_MUX_0_GND);
    ads111x_set_gain(&dev, GAIN);

    float gain_val = ads111x_gain_values[GAIN];
    int16_t raw = 0;
    gpio_config_t button_mid_config={               //按钮设置
        .intr_type = GPIO_INTR_NEGEDGE,             // 禁止中断  
        .mode = GPIO_MODE_INPUT,                    // 选择输出模式
        .pin_bit_mask = 1ULL<<33,                   // 配置GPIO_IN寄存器，
        .pull_down_en = 0,                          // 下拉电阻，1为启用
        .pull_up_en = 0,                            // 上拉电阻，1为启用
    };
    esp_err_t result;
    result = gpio_config(&button_mid_config);       //配置GPIO2  
    if (result == ESP_OK){
        printf("button_mid_config succeed \n");
    }
    else 
        printf("button_mid_config failed \n");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    float tempture=0;
    step1:
    printf("press mid button to start/stop measure.\n");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    while(gpio_get_level(33)){
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int64_t time=esp_timer_get_time();
    int64_t time_last;
    while (1)
    {
        // wait for conversion end
        bool busy;
        do
        {
            ads111x_is_busy(&dev, &busy);
        }
        while (busy);

        // Read result
        ads111x_set_input_mux(&dev, ADS111X_MUX_0_1);
        if (ads111x_get_value(&dev, &raw) == ESP_OK)
        {
            float voltage = gain_val / ADS111X_MAX_VALUE * raw;
            time_last=esp_timer_get_time();
            tempture=-50+62.5*(voltage-0.4);
            printf("time=%.3f\n",(time_last-time)/1000000.0);
            printf("A2 TO A3 Raw ADC value: %d, voltage: %.05f volts,tempture: %0.5f\n", raw, voltage,tempture);
        }
        else
            printf("Cannot read ADC value\n");
        do
        {
            ads111x_is_busy(&dev, &busy);
        }
        while (busy);
        vTaskDelay(130 / portTICK_PERIOD_MS);
        // Read result
        ads111x_set_input_mux(&dev, ADS111X_MUX_2_3);
        if (ads111x_get_value(&dev, &raw) == ESP_OK)
        {
            float voltage = gain_val / ADS111X_MAX_VALUE * raw;
            time_last=esp_timer_get_time();
            tempture=-50+62.5*(voltage-0.4);
            //printf("time=%.3f\n",(time_last-time)/1000000.0);
            printf("A0 TO A2 Raw ADC value: %d, voltage: %.05f volts,tempture: %0.5f\n", raw, voltage,tempture);
        }
        else
            printf("Cannot read ADC value\n");

        printf("\n");

        if(gpio_get_level(33)==0){
            goto step1;
        }
        vTaskDelay(870 / portTICK_PERIOD_MS);
    }
}
