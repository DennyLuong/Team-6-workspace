/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "CommandInterpreter.h"

void UArtInit(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    
    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART1); //enable the UART interrupt
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void ADCInit(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0  );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1  );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2  );
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3,ADC_CTL_CH3  |ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
}

char commandCharCount = 0; 
void UARTIntHandler(void){
	volatile uint32_t status = UARTInitStatus(UART1_BASE, true);  
	volatile uint32_t commandChar; 
	uint32_t * commandChars[2]; 
	char command; 

	UARTInitClear(UART1_BASE);
	while(UARTCharsAvail(UART1_BASE)){
		commandChar = UARTCharGetNonBlocking(UART1_BASE);
		UARTCharPutNonBlocking(UART1_BASE, commandChar); 
		commandChars[commandCharCount++] = commandChar; 
		if(commandCharCount == 2){
			commandCharCount = 0;
			command = CI_set_command(commandChars); 
			CI_get_response(command); 
			CI_call_function(command); 

		}
	SysCtlDelay(SysCtlClockGet() / (1000 * 3));	
	}
}

int main(void) {
	UArtInit(); 
	ADCInit(); 

    UARTCharPut(UART1_BASE, 'E');
    UARTCharPut(UART1_BASE, 'n');
    UARTCharPut(UART1_BASE, 't');
    UARTCharPut(UART1_BASE, 'e');
    UARTCharPut(UART1_BASE, 'r');
    UARTCharPut(UART1_BASE, ' ');
    UARTCharPut(UART1_BASE, 'T');
    UARTCharPut(UART1_BASE, 'e');
    UARTCharPut(UART1_BASE, 'x');
    UARTCharPut(UART1_BASE, 't');
    UARTCharPut(UART1_BASE, ':');
    UARTCharPut(UART1_BASE, ' ');

    while (1)
        ;
	
	return 0;
}
