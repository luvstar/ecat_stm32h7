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
uint32_t CalcMotion(int axis);
void ec_valinit();

#endif /* INC_FUNC_H_ */
