/*
 * motor.h
 *
 *  Created on: May 7, 2019
 *      Author: HuuNgan
 */

#ifndef MOTOR_L298_H_
#define MOTOR_L298_H_


//*****************************************************************************
//This program using these Pins:
//PC4, PC5, PC6, PC7, PD0, PD1
//If you want to change PORT, remember to check PWM_GEN.

#include <stdint.h>

#define MOTOR_PORT       GPIO_PORTB_BASE
#define motorA_Pin1      GPIO_PIN_0
#define motorA_Pin2      GPIO_PIN_1
#define motorB_Pin1      GPIO_PIN_2
#define motorB_Pin2      GPIO_PIN_3
#define MOTOR_PERIPH     SYSCTL_PERIPH_GPIOB

#define PWM_PORT         GPIO_PORTD_BASE
#define PWM_Pin_A        GPIO_PIN_0
#define PWM_Pin_B        GPIO_PIN_1
#define PWM_PERIPH       SYSCTL_PERIPH_GPIOD
#define PWM_A_Config     GPIO_PD0_M1PWM0
#define PWM_B_Config     GPIO_PD1_M1PWM1
#define PWM_OUT_A        PWM_OUT_0
#define PWM_OUT_B        PWM_OUT_1
#define PWM_OUT_A_BIT    PWM_OUT_0_BIT
#define PWM_OUT_B_BIT    PWM_OUT_1_BIT

#define PWM_DUTY_MAX     1000
#define PWM_FREQ         40000
#define MAX_PULSE_WIDTH  (SysCtlClockGet()/PWM_FREQ-1)
//SysCtlPWMClockGet() return the value of the divider which locating in 16 high significant bits

#define backward  'b'
#define forward  'f'

//*****************************************************************************
void motorA_run(char direction);
void motorB_run(char direction);
void release_A(void);
void release_B(void);
void setSpeed_A(uint16_t speed);
void setSpeed_B(uint16_t speed);

void PWM_Config(void);
void MOTOR_Config(void);


#endif /* MOTOR_L298_H_ */
