#include "motor_driver.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

static const char *TAG = "motor_driver";

/* -------- PIN DEFINITIONS -------- */
// Front Left
#define FL_IN1 26
#define FL_IN2 25
#define FL_PWM 27

// Rear Left
#define RL_IN1 13
#define RL_IN2 14
#define RL_PWM 12

// Front Right
#define FR_IN1 18
#define FR_IN2 33
#define FR_PWM 23

// Rear Right
#define RR_IN1 22
#define RR_IN2 21
#define RR_PWM 19

// Standby
#define STBY 32

#define PWM_FREQ_HZ 20000 // 20KHZ (quiet)
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1 << 10) - 1) // left shift the 1 upto 10 i.e 1024

#define MAX_WHEEL_RAD_S 1.0
#define MIN_DUTY 0.25 // 25% minimum torque
// Speeds below this are treated as zero to avoid small PWM draw and voltage dip
#define LOW_SPEED_CUTOFF_RAD_S 0.20

static void set_dir(int int1, int int2, int dir)
{
    if (dir > 0) // forward
    {
        gpio_set_level(int1, 1);
        gpio_set_level(int2, 0);
    }
    else if (dir < 0) // backward
    {
        gpio_set_level(int1, 0);
        gpio_set_level(int2, 1);
    }
    else
    {
        gpio_set_level(int1, 0);
        gpio_set_level(int2, 0);
    }
}

static void set_pwm(ledc_channel_t ch, double duty)
{
    // clamp the duty in 0.0->1.0
    duty = fmin(fmax(duty, 0.0), 1.0); // -ve -->0.0 and limit to 1.0
    uint32_t pwm = (uint32_t)(duty * PWM_MAX_DUTY);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, pwm); // write the pwm in register
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);   // update the register with new pwm
}

void motor_driver_init(void)
{
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << FL_IN1) | (1ULL << FL_IN2) | // pin ko bit main store karta apne pass 36 pin hai isliye 1 unsigned Long Long ko left shift kar rhe bit lane ke liye
            (1ULL << RL_IN1) | (1ULL << RL_IN2) |
            (1ULL << FR_IN1) | (1ULL << FR_IN2) |
            (1ULL << RR_IN1) | (1ULL << RR_IN2) |
            (1ULL << STBY)};
    gpio_config(&io);
    gpio_set_level(STBY, 1); // enable TB6612

    // pwm timer ----> kitna hz ,freq sara kuch qki pwm on off he toh hai resolution-->kitna step mai , freq->kitna fast, isliye isme timer ka jarurat hai
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .duty_resolution = PWM_RESOLUTION,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    // PWM channels : hardware channel jaha pwm banta hai
    ledc_channel_config_t ch[] = {
        {.gpio_num = FL_PWM,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_0,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},
        {.gpio_num = RL_PWM,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_1,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},
        {.gpio_num = FR_PWM,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_2,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},
        {.gpio_num = RR_PWM,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_3,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0}

    };
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config(&ch[i]);
    }

    ESP_LOGI(TAG, "TB6612 motor driver initialized");
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
void motor_driver_set_velocity(double left_rad_s, double right_rad_s)
{
    left_rad_s = clamp(left_rad_s, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    right_rad_s = clamp(right_rad_s, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);

    // Drop very small speeds to zero so we do not energize the bridge for negligible motion
    if (fabs(left_rad_s) < LOW_SPEED_CUTOFF_RAD_S)
        left_rad_s = 0.0;
    if (fabs(right_rad_s) < LOW_SPEED_CUTOFF_RAD_S)
        right_rad_s = 0.0;

    double left_duty = fabs(left_rad_s) / MAX_WHEEL_RAD_S;
    double right_duty = fabs(right_rad_s) / MAX_WHEEL_RAD_S;

    if (left_duty > 0.0 && left_duty < MIN_DUTY)
        left_duty = MIN_DUTY;

    if (right_duty > 0.0 && right_duty < MIN_DUTY)
        right_duty = MIN_DUTY;

    int left_dir = (left_rad_s > 0) - (left_rad_s < 0);
    int right_dir = (right_rad_s > 0) - (right_rad_s < 0);
    ESP_LOGI(TAG, "left: %.2f , right: %.2f" , left_rad_s,right_rad_s);

    //ESP_LOGI(TAG, "left dir : %d , right dir: %d", left_dir, right_dir);
    set_dir(FL_IN1, FL_IN2, left_dir);
    set_dir(RL_IN1, RL_IN2, left_dir);
    set_pwm(LEDC_CHANNEL_0, left_duty);
    set_pwm(LEDC_CHANNEL_1, left_duty);

    set_dir(FR_IN1, FR_IN2, right_dir);
    set_dir(RR_IN1, RR_IN2, right_dir);
    set_pwm(LEDC_CHANNEL_2, right_duty);
    set_pwm(LEDC_CHANNEL_3, right_duty);
}
void motor_driver_stop(void)
{
    set_pwm(LEDC_CHANNEL_0, 0);
    set_pwm(LEDC_CHANNEL_1, 0);
    set_pwm(LEDC_CHANNEL_2, 0);
    set_pwm(LEDC_CHANNEL_3, 0);
    ESP_LOGI(TAG, "MOTOR STOP");
}
void motor_driver_test(void)
{
    gpio_set_level(STBY, 1);

    set_dir(FL_IN1, FL_IN2, 1);
    set_dir(RL_IN1, RL_IN2, 1);
    set_dir(FR_IN1, FR_IN2, 1);
    set_dir(RR_IN1, RR_IN2, 1);

    set_pwm(LEDC_CHANNEL_0, 0.5);
    set_pwm(LEDC_CHANNEL_1, 0.5);
    set_pwm(LEDC_CHANNEL_2, 0.5);
    set_pwm(LEDC_CHANNEL_3, 0.5);

    ESP_LOGW(TAG, "FORCED MOTOR TEST");
}
