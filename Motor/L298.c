//*****************************************************************************
//  motor.c
//*---------------------------DC MOTOR CONTROL---------------------------------
//--------------------------Pay It Forward Club--------------------------------
//-------------------------Author: LUONG HUU NGAN------------------------------
//-------------------------Created on: May 7, 2019-----------------------------
//-------------------------Feel Free to copy it--------------------------------
//*****************************************************************************
#include <Motor/L298.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"

//*****************************************************************************
//Speed range is from 1 to 100.
void setSpeed_A(uint16_t speed)
{
//    speed = speed*0.6+400;
    if(speed > PWM_DUTY_MAX) speed = PWM_DUTY_MAX;
    PWMPulseWidthSet( PWM1_BASE, PWM_OUT_A, 1 + speed*MAX_PULSE_WIDTH/PWM_DUTY_MAX);
}

void setSpeed_B(uint16_t speed)
{
//    speed = speed*0.6+400;
    if(speed > PWM_DUTY_MAX) speed = PWM_DUTY_MAX;
    PWMPulseWidthSet( PWM1_BASE, PWM_OUT_B, 1 + speed*MAX_PULSE_WIDTH/PWM_DUTY_MAX);
}

void motorA_run(char direction)
{
    if (direction==forward)
    {
        GPIOPinWrite(MOTOR_PORT, motorA_Pin1, motorA_Pin1);
        GPIOPinWrite(MOTOR_PORT, motorA_Pin2, 0x00);
    }
    else if (direction==backward)
    {
        GPIOPinWrite(MOTOR_PORT, motorA_Pin1, 0x00);
        GPIOPinWrite(MOTOR_PORT, motorA_Pin2, motorA_Pin2);
    }
}

void motorB_run(char direction)
{
    if (direction==forward)
    {
        GPIOPinWrite(MOTOR_PORT, motorB_Pin1, motorB_Pin1);
        GPIOPinWrite(MOTOR_PORT, motorB_Pin2, 0x00);
    }
    else if (direction==backward)
    {
        GPIOPinWrite(MOTOR_PORT, motorB_Pin1, 0x00);
        GPIOPinWrite(MOTOR_PORT, motorB_Pin2, motorB_Pin2);
    }
}

void release_A(void)
{
    GPIOPinWrite(MOTOR_PORT, motorA_Pin1|motorA_Pin2, 0x00);
}

void release_B(void)
{
    GPIOPinWrite(MOTOR_PORT, motorB_Pin1|motorB_Pin2, 0x00);
}

void PWM_Config(void)
{
    //Divide system clock by 1
    SysCtlPWMClockSet( SYSCTL_PWMDIV_1 );

    //Enable the PWM peripheral, where we select PWM1 module.
    SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM1 );

    // Enable the peripheral access. PD0 is M1PWM0
    SysCtlPeripheralEnable(PWM_PERIPH);

    //Select the Pin Type for PWM
    GPIOPinTypePWM(PWM_PORT,PWM_Pin_A | PWM_Pin_B);

    //M1PWM0 corresponds to Motion Control Module 1 PWM 0.
    //This signal is controlled by Module 1 PWM Generator 0.
    //Similar to M1PWM1
    GPIOPinConfigure( PWM_A_Config );
    GPIOPinConfigure( PWM_B_Config );

    //This function is used to set the mode of operation for a PWM generator which can be counting mode,
    //synchronization mode, also a PWM generator can count in two different modes:
    //countdown mode or count up/down mode.
    PWMGenConfigure( PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN );
    PWMGenPeriodSet( PWM1_BASE, PWM_GEN_0, MAX_PULSE_WIDTH);     //Total Pulse Period

    //Set speed to 1 to stop motor
    //If you set speed to 0, it will work the same as max value.
    PWMPulseWidthSet( PWM1_BASE, PWM_OUT_A,  1 );
    PWMPulseWidthSet( PWM1_BASE, PWM_OUT_B,  1 );

    //Turn on the OUTPUT Pins
    PWMOutputState( PWM1_BASE, PWM_OUT_A_BIT | PWM_OUT_B_BIT, true );

    //Enable the PWM generator
    PWMGenEnable( PWM1_BASE, PWM_GEN_0 );

}

void MOTOR_Config(void)
{
    PWM_Config();
    //Enable motor peripheral
    SysCtlPeripheralEnable(MOTOR_PERIPH);

    // Enable the GPIO pin for the motors.  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(MOTOR_PORT, motorA_Pin1|motorA_Pin2|motorB_Pin1|motorB_Pin2);
}
