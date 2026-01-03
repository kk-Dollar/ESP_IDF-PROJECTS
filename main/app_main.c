#include "microros_node.h"
#include "control_loop.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define CONTROL_LOOP_HZ 100
#define CONTROL_DT_SEC (1.0 / CONTROL_LOOP_HZ) // passes to control_loop to find position or kinematics dt p=v*dt;

//static const char *TAG = "app_main";

static void micro_ros_task(void *arg)
{
    (void)arg;

    microros_init();

    while (1)
    {
        microros_spin();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void control_loop_task(void *arg)
{
    (void)arg;

    control_loop_init();

    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ); // freertos uses ticks time unit 1 ticks = 1 ms, so period = 10 tick means 10ms. means control loop will be run on every 10ms or 10 tick ; 1s=1000ms
    TickType_t last_wake = xTaskGetTickCount();                      // used as referece to find : when to wake up last_wake + period

    while (1)
    {
        control_loop_update(CONTROL_DT_SEC);
        vTaskDelayUntil(&last_wake, period); // why not only vtaskdelay bcz: suppose control_loop take 3ms then in case of vtaskdelay 3+10 but in this case only 10ms in every case if control take 3ms then task sleep for 7ms
    }
}

void app_main(void)
{
    nvs_flash_init();

    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for WiFi to connect

    uros_network_interface_initialize();

    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 5, NULL);
    xTaskCreate(control_loop_task,"control_loop_task",4096,NULL,6,NULL); //slightly higher priority than ros
}