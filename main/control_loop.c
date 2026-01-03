#include "control_loop.h"
#include "microros_node.h"
#include "motor_driver.h"
#include "esp_log.h"

static const char *TAG = "control_loop";

// for accelerating limiting
static double prev_left = 0.0;
static double prev_right = 0.0;

void control_loop_init(void)
{
    prev_left = 0.0;
    prev_right = 0.0;
    
    motor_driver_init();
    ESP_LOGI(TAG, "Control loop initialised");
}

void control_loop_update(double dt)
{

    (void)dt; // unused now, will be used later

    double left_cmd = microros_get_left_cmd();
    double right_cmd = microros_get_right_cmd();
    // ESP_LOGI(TAG,
    //          "CTRL | left: %.3f  right: %.3f",
    //          left_cmd, right_cmd);

    motor_driver_set_velocity(left_cmd,right_cmd);         

    /* Save for future filters */
    prev_left = left_cmd;
    prev_right = right_cmd;

    /*
     * NEXT STEPS (later):
     *  - acceleration limiting
     *  - PID
     *  - motor_set_velocity(left_cmd, right_cmd)
     */
}