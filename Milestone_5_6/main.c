/*
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "driverlib/debug.h"
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

	//UARTCharPut(UART1_BASE, 0x3A); // :
	//UARTCharPut(UART1_BASE, 0x20); // space
	UARTprintf("\n");
	UARTprintf(LookupTable[index]);

	UARTCharPut(UART1_BASE, 0x20); 	// space

	UARTprintf(LookupTable[index + 1]);
	UARTCharPut(UART1_BASE, 0x20);	// space
	UARTCharPut(UART1_BASE, 0x0D);	// ??
	UARTCharPut(UART1_BASE, 0x0A);	// ??

	return index;
}


void frontDistance(void){
	uint32_t ui32ADC0Value[4];
	volatile uint32_t ui32DistAvg;

	// Clear interrupt
	ADCIntClear(ADC0_BASE, 1);
	ADCProcessorTrigger(ADC0_BASE, 1);	// Trigger ADC Conversion
	// Wait on conversion to finish
	while(!ADCIntStatus(ADC0_BASE, 1, false))
	{
	}
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	ui32DistAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;

	// FIX: output avg distance to terminal
	// FIX: fix range value for inches or centimeters (OPTIONAL)

	UARTprintf("Distance: ");
	UARTprintf("%d \n",ui32DistAvg);

	SysCtlDelay(6700000);
}


unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;

void PWMForward(void){
	// forward
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
	int speedAdjust;

	UARTprintf("Enter Speed(1-100): ");

	UARTgets(inSpeed, 4);

	speedAdjust = atoi(inSpeed);		//conversion from ASCII to INT

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speedAdjust * ui32Load/100);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speedAdjust * ui32Load/100);
}

void getFunction(char function){
	// MAIN COMMAND SELECTOR
	switch (function) {
	case R0:
		frontDistance();
		break;
	case MF:
		PWMForward();
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
	default:
		UARTprintf("Error \n");
		break;
	}

}


void UARTIntHandler(void)
{
	char characters[2] = " ";
	char function = 0;

	uint32_t ui32Status;

	ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status

	UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts

	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
	UARTgets(characters, 3);
	//SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
	UARTprintf("\n");
	//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

	function = responseGet(characters);
	getFunction(function);
	UARTprintf("Enter Command: ");





}









void UARTInit(void){
	// Enable UART & GPIO peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Configuration of GPIO pins and types
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// set clock and standard I/O
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, 115200, 16000000);
	//UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

	//UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void ADCInit(void) {
	// Enable ADC and GPIO E PIN 2 for ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	// Set sequence and configure steps
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0  );
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1  );
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2  );
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3,ADC_CTL_CH3  |ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
}



void PWMInit(void)
{
	//SET PWM CLOCK
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	// Enable Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	// Assign GPIOs Types
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);	// RIGHT MOTOR
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);	// LEFT MOTOR
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2);	// RIGHT PHASE
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_3);	// LEFT PHASE
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);	// MODE
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

	// Enable MODE = 1
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
	//PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}


int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // System clock 40MHz
	UARTInit();
	//ADCInit();
	PWMInit();

	IntMasterEnable(); 	// Enable processor interrupts
	IntEnable(INT_UART1);	// Enable the UART interrupt
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts

	UARTprintf("Embedded Systems - TEAM 6 \r\n");
	UARTprintf("Enter Command: ");


	while (1)
	{
		/*//uint32_t ui32Status;
		//uint32_t character;
		char characters[2] = " ";
		char function = 0;


		UARTgets(characters, 3);
		//UARTprintf(characters);
		UARTprintf("\n");


		//ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
		function = responseGet(characters);
		getFunction(function);
		UARTprintf("Enter Command: ");


		SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
		//UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts
	*/
	}
}
