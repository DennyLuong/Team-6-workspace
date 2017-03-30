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

#define PWM_FREQUENCY 55

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

	UARTCharPut(UART1_BASE, 0x20); 	// space

	UARTprintf(LookupTable[index + 1]);
	UARTCharPut(UART1_BASE, 0x20);	// space
	UARTCharPut(UART1_BASE, 0x0D);	// ??
	UARTCharPut(UART1_BASE, 0x0A);	// ??

	return index;
}

/**************************************
 *     	    Command Functions         *
 **************************************/

uint32_t rightDistance(void){
	uint32_t ui32ADC0Value[4];
	volatile uint32_t ui32DistAvg;

	// clear interrupt
	ADCIntClear(ADC0_BASE, 1);
	// trigger ADC conversion
	ADCProcessorTrigger(ADC0_BASE, 1);
	// wait for conversion to complete
	while(!ADCIntStatus(ADC0_BASE, 1, false))
	{
	}
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	ui32DistAvg = ui32ADC0Value[0];

//	UARTprintf("Distance: ");
//	UARTprintf("%u \n",ui32DistAvg);

	return ui32DistAvg;
}


unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;
int speedAdjust;

void PWMForward(void){
	ui8Adjust = ui32Load;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 12);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust);
}

void PWMStart(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
}

void PWMStop(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), false);
}

void PWMSpeed(void){

	char inSpeed[3];

	UARTprintf("Enter Speed (1-100): ");

	UARTgets(inSpeed, 4);

	speedAdjust = atoi(inSpeed);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speedAdjust * ui32Load/100);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speedAdjust * ui32Load/100);
}

void PWMRightReverse(void){
	ui8Adjust = ui32Load;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 4);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust);
}

void PWMLeftReverse(void){
	ui8Adjust = ui32Load;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 8);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust);
}

void setSpeed(unsigned int adjust) {

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, adjust);
}

void rotateCW(int diff) {

	unsigned int speed = diff * ui32Load/100;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 4);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speed);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speed);
}

void rotateCCW(int diff) {

	unsigned int speed = diff * ui32Load/100;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 8);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speed);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speed);
}

/**************************************
 *  	  PID Control Functions       *
 **************************************/

void PIDStart(void) {
	// enable Timer0, SubtimerA
	TimerEnable(TIMER0_BASE, TIMER_A);
	unsigned int speed = 2840;
	setSpeed(speed);
	PWMStart();
	PWMForward();
}

uint32_t cur_pos;
const uint32_t target_dist = 1000;
int proportional;
int last_proportional = 0;
int derivative;
int integral = 0;
const float p_const = 20;
const int i_const = 10000;
float d_const = 1.5;
const int max = 100;

void computePID(void){

	// PID calculations
	cur_pos = rightDistance();
	proportional = cur_pos - target_dist;
	//derivative = proportional - last_proportional;
	//integral += proportional;
	//last_proportional = proportional;

	int power_difference = proportional/p_const;

	UARTprintf("Current Position: %u Proportional: %d Power Difference: %d\n", cur_pos, proportional, power_difference);

	if (power_difference > max)
		power_difference = max;
	if (power_difference < -max)
		power_difference = -max;

	if (power_difference < 0) {
		rotateCW(power_difference);
	}
	else if (power_difference > 80) {
		rotateCCW(power_difference);
	}
	else {
		PWMForward();
	}

}

/**************************************
 *     	    Command Functions         *
 **************************************/

/*
 *  getFunction
 *  - Main command selector
 */
void getFunction(char function){
	switch (function) {
	case R1:
		rightDistance();
		break;
	case MF:
		PWMForward();
		break;
	case RR:
		PWMRightReverse();
		break;
	case LR:
		PWMLeftReverse();
		break;
	case ES:
		PWMStop();
		break;
	case SS:
		PWMSpeed();
		break;
	case MS:
		PWMStart();
		break;
	case GO:
		PIDStart();
		break;
	default:
		UARTprintf("Error \n");
		break;
	}

}

/*
 * UART Interrupt Handler
 */
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

/*
 * Timer0 Interrupt Handler
 */

void Timer0IntHandler(void) {

	// clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	computePID();

}

/**************************************
 * Interface Initialization Functions *
 **************************************/

void UARTInit(void){

	// enable UART & GPIO peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// configuration of GPIO pins and types
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// set clock and standard I/O
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, 115200, 16000000);

}

void ADCInit(void) {

	// enable ADC0 and GPIOE Pin 2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

	// set sequence and configure steps
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
}

void PWMInit(void)
{

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	// enable PWM1, GPIOD and GPIOE
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	// assign PWM type to GPIO Pins
	// D0: right motor		D2: right phase
	// D1: left motor		D3: left phase
	// E1: mode
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	// enable mode = 1
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
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

	// set system clock -> 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	UARTInit();
	ADCInit();
	PWMInit();
	TimerInit();

	// enable processor interrupts
	IntMasterEnable();
	// enable the UART interrupt
	IntEnable(INT_UART1);
	//only enable RX and TX interrupts
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

	UARTprintf("Embedded Systems - TEAM 6 \r\n");
	UARTprintf("Enter Command: ");


	while (1)
	{

	}
}
