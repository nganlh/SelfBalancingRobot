#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "debug.h"
#include "I2C/mpu6050.h"
#include "Motor/L298.h"

//***Compiler Option***********************************************************
//#define     CALIB
//#define     DEBUG_UART
//*****************************************************************************

//***Definitions***************************************************************
#define MOTOR_A_OFFSET           468
#define MOTOR_B_OFFSET           475
#define MAX_ANGLE                60
#define TIMER_PERIOD                (SysCtlClockGet()/50)
#define GYRO_DRIFTING_CONST     (float)(0.00118)
#define NUM_OF_VALUES(a) ((unsigned long long)1 << \
        (sizeof(a) * 8))
//*****************************************************************************

typedef struct
{
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} OffsetCalibration;

typedef struct
{
    float Kp, Ki, Kd, current_err, previous_err;
    float iteration_sum, iteration_time;
}PID_coefficient;

int16_t accaxisX, accaxisY, accaxisZ, gyroaxisX, gyroaxisY, gyroaxisZ;
int16_t motor_duty;
uint8_t timer_count=0, timer_count_pre=0;
float acc_angle;
float gyroAngle = 0;
float gyroRate;
float filted_Angle;

OffsetCalibration OFFSET = {62, -11, 32, 185, 8, 3740};
PID_coefficient MotorPID = {25, 0.00008, 20};

//**********DEBUG**************************************************************
#ifdef DEBUG_UART
void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
#endif
//*****************************************************************************

//**********Calibration mode***************************************************
#ifdef CALIB
#define NUM_OF_SAMPLES          150

void Calib_IMU(void)
{
    int16_t buffer[NUM_OF_SAMPLES];
    uint8_t i;
    int32_t sum;

//    SysCtlDelay(SysCtlClockGet()/6);
//    for(i=0; i<NUM_OF_SAMPLES; i++)
//    {
//        getMPU6050Data();
//        buffer[i] = accaxisX;
//        SysCtlDelay(SysCtlClockGet()/150);
//    }
//    sum=0;
//    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
//    OFFSET.acc_x = (float)sum/NUM_OF_SAMPLES;

    for(i=0; i<NUM_OF_SAMPLES; i++)
    {
        getMPU6050Data();
        buffer[i] = accaxisY;
//        SysCtlDelay(SysCtlClockGet()/150);
    }
    sum=0;
    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
    OFFSET.acc_y = sum/NUM_OF_SAMPLES;

//    for(i=0; i<NUM_OF_SAMPLES; i++)
//    {
//        getMPU6050Data();
//        buffer[i] = accaxisZ;
//        SysCtlDelay(SysCtlClockGet()/150);
//    }
//    sum=0;
//    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
//    OFFSET.acc_z = sum/NUM_OF_SAMPLES;
//



    for(i=0; i<NUM_OF_SAMPLES; i++)
    {
        getMPU6050Data();
        buffer[i] = gyroaxisX;
//        SysCtlDelay(SysCtlClockGet()/150);
    }
    sum=0;
    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
    OFFSET.gyro_x = sum/NUM_OF_SAMPLES;

//    for(i=0; i<NUM_OF_SAMPLES; i++)
//    {
//        getMPU6050Data();
//        buffer[i] = gyroaxisY;
//        SysCtlDelay(SysCtlClockGet()/150);
//    }
//    sum=0;
//    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
//    OFFSET.gyro_y = (float)sum/NUM_OF_SAMPLES;
//
//    for(i=0; i<NUM_OF_SAMPLES; i++)
//    {
//        getMPU6050Data();
//        buffer[i] = gyroaxisZ;
//        SysCtlDelay(SysCtlClockGet()/150);
//    }
//    sum=0;
//    for(i=0; i<NUM_OF_SAMPLES; i++) sum += buffer[i];
//    OFFSET.gyro_z = sum/NUM_OF_SAMPLES;
}
#endif
//*****************************************************************************

float convertToDegree(int16_t accelValue)
{
    return (float)accelValue*90/MPU6050_ACC_SCALE_FACTOR_8;
}

float getTimeValue(void)
{
    int32_t time_duration, timerValue;
    static int previousTimerValue=0;
    timerValue = TimerValueGet(TIMER0_BASE, TIMER_A);
    if(timer_count >= timer_count_pre)
    {
        time_duration = timerValue - previousTimerValue + (timer_count - timer_count_pre)*TIMER_PERIOD;
    }
    else
    {
        time_duration = timerValue - previousTimerValue +
        (NUM_OF_VALUES(timer_count) + timer_count - timer_count_pre)*TIMER_PERIOD;
    }
    previousTimerValue = timerValue;
    timer_count_pre = timer_count;
    return (float)time_duration/(float)(SysCtlClockGet()/1000);      //return in ms
}

void timerInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_PERIOD-1);  //20ms

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
}

void Timer0AIntHandler(void)
{
    getMPU6050Data();
    gyroRate = (float)(gyroaxisX - OFFSET.gyro_x)/MPU6050_GYRO_SCALE_FACTOR_2000;
    gyroAngle += (gyroRate*0.02f - GYRO_DRIFTING_CONST);
    timer_count++;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    #ifdef  DEBUG_UART
        InitConsole();
        DBG("\nDebug is ON\n\n");
    #endif

        initI2C();
        initMPU6050();
    #ifdef CALIB
        Calib_IMU();
    #endif
        timerInit();
        MOTOR_Config();

    #ifdef DEBUG_UART
        DBG("acc_x_offset = %d\n", (int)OFFSET.acc_x);
        DBG("acc_y_offset = %d\n", (int)OFFSET.acc_y);
        DBG("acc_z_offset = %d\n", (int)OFFSET.acc_z);
        DBG("gyro_x_offset = %d\n", (int)OFFSET.gyro_x);
        DBG("gyro_y_offset = %d\n", (int)OFFSET.gyro_y);
        DBG("gyro_z_offset = %d\n", (int)OFFSET.gyro_z);
    #endif

    while(1)
    {
//        getMPU6050Data();         //Called in timer0 Interrupt Handler
        acc_angle = convertToDegree(accaxisY - OFFSET.acc_y);
        filted_Angle = Comp_Filter(&gyroAngle, &acc_angle);
        if(filted_Angle > 0 && filted_Angle < MAX_ANGLE)
        {
            motorA_run(backward);
            motorB_run(backward);
        }
        else if(filted_Angle < 0 && filted_Angle > -MAX_ANGLE)
        {
            motorA_run(forward);
            motorB_run(forward);
        }
        else
        {
            release_A();
            release_B();
        }

        //***PID controll******************************************************
        MotorPID.iteration_time = getTimeValue();
        MotorPID.current_err = filted_Angle;
        MotorPID.iteration_sum += MotorPID.current_err*MotorPID.iteration_time;
        motor_duty =MotorPID.Kp*MotorPID.current_err + MotorPID.Ki*MotorPID.iteration_sum
                        + MotorPID.Kd*(MotorPID.current_err-MotorPID.previous_err)/MotorPID.iteration_time;
        if(motor_duty >= 0)
        {
            setSpeed_A(motor_duty + MOTOR_A_OFFSET);
            setSpeed_B(motor_duty + MOTOR_B_OFFSET);
        }
        else
        {
            setSpeed_A(-motor_duty + MOTOR_A_OFFSET);
            setSpeed_B(-motor_duty + MOTOR_B_OFFSET);
        }
        MotorPID.previous_err = MotorPID.current_err;
        //*********************************************************************

        #ifdef DEBUG_UART
            UARTprintf("acc_angle = %d\n", (int)acc_angle);
            UARTprintf("gyro_angle_x = %d\n", (int)gyroAngle);
            UARTprintf("Comp_Angle_x = %d\n", (int)filted_Angle);
//            SysCtlDelay(SysCtlClockGet()/6);
        #endif
    }
}
