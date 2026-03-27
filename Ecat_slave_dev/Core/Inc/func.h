/*
 * func.h
 *
 *  Created on: Mar 18, 2026
 *      Author: cubox
 */

#ifndef INC_FUNC_H_
#define INC_FUNC_H_

#include "TMC2209_usart.h"
#include "ecat_slv.h"
#include "utypes.h"

uint32_t motCtrl();
uint8_t setEcat();
void servo_on(int axis);
void servo_off(int axis);
uint32_t get_MaxSpeed(int axis);
uint8_t mot_ProfUpdate(int axis);
uint8_t mot_MaxSpdUpdate(int axis);
void CalcMotion(int axis);
void ec_valinit();
void update_timer_pwm_freq(uint8_t axis, uint32_t output_speed_hz);
void stop_pulse_generator(uint8_t axis);
void Reset_Actual_Position(uint8_t axis);
void Update_Actual_Position(uint8_t axis);
#endif /* INC_FUNC_H_ */
