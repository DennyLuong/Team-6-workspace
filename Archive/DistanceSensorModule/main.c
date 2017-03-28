#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"

void UARTinit(void);
void ADCinit(void);
void delay(void);

int main(void) {
	uint32_t ui32ADC0Value[4];
	volatile uint32_t ui32DistAvg;
	volatile uint32_t ui32temp;
	uint8_t ui8outChar[5];
	int i = 0;
	int decimalNum = 0;
	// Sets clock
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	UARTinit();		//UART Initialize
	ADCinit();		//ADC Initialize
	


// UARTStdioConfig(UART0_BASE, 115200, SysCtlClockGet());        TRYING TO SET UP Standard I/O to use functions
//    UARTprintf("Enter text: ");
    while (1)
    {
    	ADCIntClear(ADC0_BASE, 1);
    	ADCProcessorTrigger(ADC0_BASE, 1);
    	//UARTStdioConfig(UART0_BASE, 115200, SysCtlClockGet());

    	while(!ADCIntStatus(ADC0_BASE, 1, false))
    	{
    	}
    	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    	ui32DistAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;



    	// FIX: output avg distance to terminal
    	// FIX: fix range value for inches or centimeters (OPTIONAL)
        UARTCharPut(UART0_BASE, 'D');
        UARTCharPut(UART0_BASE, 'i');
        UARTCharPut(UART0_BASE, 's');
        UARTCharPut(UART0_BASE, 't');
        UARTCharPut(UART0_BASE, 'a');
        UARTCharPut(UART0_BASE, 'n');
        UARTCharPut(UART0_BASE, 'c');
        UARTCharPut(UART0_BASE, 'e');
        UARTCharPut(UART0_BASE, ':');
        UARTCharPut(UART0_BASE, ' ');

        ui32temp = ui32DistAvg;
        decimalNum = 0;
        for (i = 0; ui32temp >= 1; i++) {
        	ui32temp = ui32temp/10;
        	decimalNum++;
        }
        ui8outChar[0] = 'a';
        ui8outChar[1] = ' ';
        ui8outChar[2] = ' ';
        ui8outChar[3] = ' ';
        ui8outChar[4] = ' ';

        for (i = decimalNum - 1; i >= 0; i--) {
        	ui8outChar[i] = (char)48 + (ui32DistAvg % 10);
        	ui32DistAvg /= 10;
        }
        for (i = 0; i < decimalNum; i++) {
        	UARTCharPut(UART0_BASE, ui8outChar[i]);
        }

        UARTCharPut(UART0_BASE, '\r');
        UARTCharPut(UART0_BASE, '\n');

        delay();
/*
    	UARTCharPut(UART0_BASE, ui32DistAvg));
    	UARTprintf(ui32DistAvg);
*/


    }

    return 0;


}




void UARTinit(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

}

void ADCinit(void) {
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

void delay(void)
{
	 SysCtlDelay(6700000);		// creates ~500ms delay - TivaWare fxn

}
