/*
 * [ECE XX37] - Embedded Systems
 * Spring 2017
 *
 * Team 6
 * Authors: Robert Duenez, Katherine Perez, Sikender Shahid
 *
 * Milestone 6: Add PID control to robot
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include <string.h>

/********************************************************************************************************
 *                                             PWM Functions                                            *
 ********************************************************************************************************/
#define PWM_FREQUENCY 10000

unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;

void move_forward(void){
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_2|GPIO_PIN_7);
}

void move_reverse(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, 0|0);
}

void move_cw(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_2|0);
}

void move_ccw(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_7|0);
}

void PWMStart(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
	PWMSetSpeed(ui32Load);
}

void PWMStop(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), false);
}

void PWMUserSetSpeed(void){
	unsigned char inSpeed[3];
	UARTprintf("Enter Speed (1-100): ");
	UARTgets(inSpeed, 4);
	PWMSetSpeed(atoi(inSpeed));
}

void PWMSetSpeed(unsigned int adjust){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, adjust * ui32Load/100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, adjust * ui32Load/100);
}

/********************************************************************************************************
 *                                       PID Control Functions                                          *
 ********************************************************************************************************/
void PIDStart(void) {
	// enable Timer0, SubtimerA
	TimerEnable(TIMER0_BASE, TIMER_A);
	unsigned int speed = 2840;
	PWMSetSpeed(speed);
	PWMStart();
	move_forward();
}

int front_target = 2000;
uint32_t cur_pos;
const uint32_t target_dist = 1500;
int proportional;
int last_proportional = 0;
int derivative;
int integral = 0;
const float p_const = 20;
const int i_const = 3750;
float d_const = 1.5;
const int max = 100;

void computePID(void){

	// PID calculations
	while(frontDistance() > front_target)
	{
		move_ccw();
		PWMSetSpeed(80);
	}


	cur_pos = rightDistance();
	proportional = cur_pos - target_dist;
	//derivative = proportional - last_proportional;
	integral += proportional;
	//last_proportional = proportional;

	int power_difference = proportional/p_const  + integral/i_const + derivative * d_const;

	UARTprintf("Current Position: %u Proportional: %d Power Difference: %d\n", cur_pos, proportional, power_difference);

	if (power_difference > max)
		power_difference = max;
	if (power_difference < -max)
		power_difference = -max;


	if (power_difference < 0) {
		move_cw();
		PWMSetSpeed(150 + power_difference);
	}
	else if (power_difference == max) {
		move_ccw();
		PWMSetSpeed(power_difference);
	}
	else {
		move_forward();
	}

}



/*
 * Timer0 Interrupt Handler
 */

void Timer0IntHandler(void) {
	// clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	computePID();
}

/********************************************************************************************************
 *                                     ADC get_adc_value Functions                                      *
 ********************************************************************************************************/

uint8_t (*get_adc_value)(void);  // when working on the PID access using function pointer
uint32_t rightDistance(void){
    // step 0 - ADC_CTL_CH0
	uint32_t ui32ADC0Value[4];
	ADCIntClear(ADC0_BASE, 1);
	ADCProcessorTrigger(ADC0_BASE, 1);
	while(!ADCIntStatus(ADC0_BASE, 1, false)){
	}// wait for conversion to complete
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    UARTprintf("value right distance:");
    UARTprintf("%u \n", ui32ADC0Value[0]);
    return ui32ADC0Value[0];
}
uint32_t frontDistance(void){
    // step 1 - ADC_CTL_CH1
	uint32_t ui32ADC0Value[4];
	ADCIntClear(ADC0_BASE, 2);
	ADCProcessorTrigger(ADC0_BASE, 2);
	while(!ADCIntStatus(ADC0_BASE, 2, false)){
	}// wait for conversion to complete
	ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);

    UARTprintf("value front distance:");
    UARTprintf("%u \n", ui32ADC0Value[0]);

   return ui32ADC0Value[0];
}
uint8_t rightReflection(void){
    // step 2 - ADC_CTL_CH8
    UARTprintf("value right flection:");
    uint8_t value = ADC(2);
    UARTprintf("%u \n", value);
    return value;
}

uint8_t leftReflection(void){
    // step 3 - ADC_CTL_CH9
    UARTprintf("value left reflection:");
    uint8_t value = ADC(3);
    UARTprintf("%u \n", value);
    return value;
}

uint8_t ADC(uint8_t value){
    uint32_t ui32ADC0Value[4];
    ADCIntClear(ADC0_BASE, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false)){
    }// wait for conversion to complete
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    return ui32ADC0Value[value];
}
/********************************************************************************************************
 *                                     UART Handler and Command Functions                               *
 ********************************************************************************************************/

void UARTIntHandler(void)
{
    char characters[2] = " ";
    char function = 0;
    uint32_t ui32Status;
    //get interrupt status
    ui32Status = UARTIntStatus(UART1_BASE, true);
    //clear the asserted interrupts
    UARTIntClear(UART1_BASE, ui32Status);
    UARTgets(characters, 3);
    UARTprintf("\n");
    function = responseGet(characters);
    getFunction(function);
    UARTprintf("Enter Command: ");
}

typedef enum{
    HH=0,
    MS=2,
    P0=4,
    MF=6,
    RR=8,
    SS=10,
    LR=12,
    LG=14,
    GO=16,
    R0=18,
    R1=20,
    TD=22,
    DS=24,
    ES=26,
    DC=28,
    ER=30
}Commands;

//typedef void(*function)(void);
//struct commandStruct {
//    char const *name;
//    function execute;
//    char const *response;
//};
//
//const struct commandStruct commands[] = {
//    {"help", &helpCommand, "Printing Help Statement... " },
//    {"name2", &sayPoop, "poooooop"},
//};
//
//commands[0].execute();

//void helpCommand (void){
//    static const helplisting[]= "\n";
//}



uint8_t CharacterCount = 0;

char responseGet(char chars[]){
    static const char * LookupTable[] = {
            "HH", " Enter 2char function      ",
            "MS", " PWM enable                ",
            "P0", " PWM disable               ",
            "MF", " Moving Forward            ",
            "RR", " Right Wheel Reverse       ",
            "SS", " Set Speed                 ",
            "LR", " Left Wheel Reverse        ",
            "LG", " Light Get                 ",
            "GO", " PID initiated             ",
            "R0", " Read Front Distance Sensor",
            "R1", " Read Right Distance Sensor",
            "TD", " Toggle Data Acquisition   ",
            "DS", " Post Drive Semaphore      ",
            "ES", " Emergency Stop            ",
            "DC", " Drive Clock Start         ",
            "ER", " Error                     "
    };

    char index = 0;
    if(!strcmp(chars,"HH"))
        index = HH;
    else if(!strcmp(chars,"MS"))
        index = MS;
    else if(!strcmp(chars,"P0"))
        index = P0;
    else if(!strcmp(chars,"MF"))
        index = MF;
    else if(!strcmp(chars,"RR"))
        index = RR;
    else if(!strcmp(chars,"SS"))
        index = SS;
    else if(!strcmp(chars,"LR"))
        index = LR;
    else if(!strcmp(chars,"GO"))
        index = GO;
    else if(!strcmp(chars,"R0"))
        index = R0;
    else if(!strcmp(chars,"R1"))
        index = R1;
    else if(!strcmp(chars,"TD"))
        index = TD;
    else if(!strcmp(chars,"DS"))
        index = DS;
    else if(!strcmp(chars,"ES"))
        index = ES;
    else if(!strcmp(chars,"DC"))
        index = DC;
    else
        index = ER;

    UARTprintf("\n");
    UARTprintf(LookupTable[index]);

    UARTCharPut(UART1_BASE, 0x20);  // space

    UARTprintf(LookupTable[index + 1]);
    UARTCharPut(UART1_BASE, 0x20);  // space
    UARTCharPut(UART1_BASE, 0x0D);  // ??
    UARTCharPut(UART1_BASE, 0x0A);  // ??

    return index;
}

void getFunction(char function){
    switch (function) {
    case R1:
        rightDistance();
        frontDistance();
        //rightReflection();
        //leftReflection();
        break;
    case MF:
    	move_forward();
        break;
    case RR:
    	move_cw();
        break;
    case LR:
    	move_ccw();
        break;
    case ES:
        PWMStop();
        break;
    case SS:
    	PWMUserSetSpeed();
        break;
    case MS:
        PWMStart();
        break;
    case GO:
        PIDStart();
        //computePID();
        break;
    default:
        UARTprintf("Error \n");
        break;
    }
}
/********************************************************************************************************
 *                                     Interface Initialization Functions                               *
 ********************************************************************************************************/

void UARTInit(void){
    // UART Module 1
    // Port PB0 RX
    // Port PB1 TX

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// configuration of GPIO pins and types
	GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// set clock and standard I/O
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, 115200, 16000000);
	// Interrupt Enabled for Data on TX and RX
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void ADCInit(void) {
    //  Port PE3 AIN-0 - ADC_CTL_CH0 - RIGHT DISTANCE SENSOR
    //  Port PE2 AIN-1 - ADC_CTL_CH1 - FRONT DISTANCE SENSOR
    //  Port PE5 AIN-8 - ADC_CTL_CH8 - RIGHT LIGHT SENSOR
    //  Port PE4 AIN-9 - ADC_CTL_CH9 - LEFT LIGHT SENSOR

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);
	//ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH8 );
	//ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH9  | ADC_CTL_END |ADC_CTL_IE);
	ADCSequenceEnable(ADC0_BASE, 1);
	ADCSequenceEnable(ADC0_BASE, 2);
}

void PWMInit(void)
{
    //  Port PE1 - MODE = 1
    //  Port PD3 - PHASE PIN - RIGHT MOTOR  // changed to PD7
    //  Port PD1 - PWM PIN   - RIGHT MOTOR
    //  Port PD2 - PHASE PIN - LEFT MOTOR
    //  Port PD0 - PWM PIN   - LEFT MOTOR
    // FORWARD 0 BITMASKED  // LOGIC SWITCHED - > FORWARD 1 BITMASKED
    // REVERSE 1 BITMASKED  // LOGIC SWITCHED - > REVERSE 0 BITMASKED

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// hardware unlock for gpio pd7
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	// enable mode = 1
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	//PWMStart();
	//move_forward();
}

void TimerInit(void) {

	uint32_t ui32Period;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// enable timer0 as 32-bit timer in periodic mode
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	// toggle GPIO at 20Hz for 50% duty cycle
	ui32Period = (SysCtlClockGet() / 20) / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period-1);

	// enable timer0a interrupt vector
	IntEnable(INT_TIMER0A);
	// enable specific vector within timer to generate an interrupt
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	IntMasterEnable();
	UARTInit(); //Interrupt Driven
	ADCInit();
	PWMInit();
//PID
	TimerInit(); //Interrupt Driven//DMAInit(); //Interrupt Driven


//
//	IntEnable(INT_UART1);
//	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

	UARTprintf("Embedded Systems - TEAM 6\r\n");
	UARTprintf("Type help\n");

	while(1)
	    ;
}
