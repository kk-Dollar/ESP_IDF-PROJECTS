#include "motor_driver.h"
#include "esp_log.h"

static const char *TAG = "motor_driver";

#define MAX_WHEEL_RAD_S 5.0

void motor_driver_init(void)
{
    ESP_LOGI(TAG, "Motor driver initialized");
}
// clamp helper
static double clamp(double v, double min, double max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}
void motor_driver_set_velocity(double left_rad_s,double right_rad_s)
{
    left_rad_s=clamp(left_rad_s,-MAX_WHEEL_RAD_S,MAX_WHEEL_RAD_S);
    right_rad_s=clamp(right_rad_s,-MAX_WHEEL_RAD_S,MAX_WHEEL_RAD_S);

    ESP_LOGI(TAG,"MOTOR| left: %.3f rad/s right: %.3f rad/s",left_rad_s,right_rad_s);
}
void motor_driver_stop(void)
{
    ESP_LOGI(TAG,"MOTOR STOP");
}