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
#include "CommandInterpreter.h"

// #define PWM_FREQUENCY 55


void UARTInit(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, 115200, 16000000);
	IntEnable(INT_UART0);   // Enable the UART interrupt
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
}

void ADCInit(void){
//    // Enable ADC and GPIO E PIN 2 for ADC
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//
//    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
//    // Set sequence and configure steps
//    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
//    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0  );
//    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1  );
//    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2  );
//    ADCSequenceStepConfigure(ADC0_BASE, 1, 3,ADC_CTL_CH3  |ADC_CTL_IE|ADC_CTL_END);
//    ADCSequenceEnable(ADC0_BASE, 1);
}

void PWMInit(void){
//    //SET PWM CLOCK
//    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
//    // Enable Peripherals
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//    // Assign GPIOs Types
//    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);    // LEFT | RIGHT Motors
//    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_1);  // RIGHT PHASE | LEFT PHASE | MODE
//    GPIOPinConfigure(GPIO_PD0_M1PWM0);
//    GPIOPinConfigure(GPIO_PD1_M1PWM1);
//
//    ui32PWMClock = SysCtlClockGet() / 64;
//    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
//    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
//    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
//
//    // Enable MODE = 1
//    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);
//
//    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
//    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
//    //PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
//    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void UARTIntHandler(void)
{
    char characters[2] = " ";
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
	UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
	UARTgets(characters, 3);
	UARTprintf("\n");
	UARTprintf(responseGet(characters));
	callFunction();
	UARTprintf("Enter Command: ");
}

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // System clock 40MHz
	IntMasterEnable();

	UARTInit();

	UARTprintf("Embedded Systems - TEAM 6\n");
	UARTprintf("Enter Command: ");
	while (1)
	{
	}
}
