#include <stdint.h>
#pragma once


 
void microros_init(void);
void microros_spin(void);
double microros_get_left_cmd(void);
double microros_get_right_cmd(void);
int64_t microros_last_cmd_age_ms(void);