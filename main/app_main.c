#include "microros_node.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "app_main";

static void micro_ros_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(3000));  // Wait for WiFi to connect
    
    microros_init();

    while(1)
    {
        microros_spin();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

void app_main(void)
{
    nvs_flash_init();
    
    uros_network_interface_initialize();
    
    xTaskCreate(micro_ros_task,"micro_ros_task",8192,NULL,5,NULL);

}