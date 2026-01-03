# pragma once

void motor_driver_init(void);
void motor_driver_set_velocity(double left_rad_s, double right_rad_s);
void motor_driver_stop(void);