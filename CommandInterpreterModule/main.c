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
#include "driverlib/adc.h"


typedef enum {
    HH = 0,
    P1 = 4,
    P0 = 8,
    RF = 12,
    RR = 16,
    LF = 20,
    LR = 24,
    LG = 28,
    GO = 32,
    R0 = 36,
    R1 = 40,
    TD = 44,
    DS = 48,
    ES = 52,
    DC = 56,
    ER = 60
}Commands;



uint8_t CharacterCount = 0;

char responseGet(uint32_t chars[]){
    const char LookupTable[] = {
        'H','H','1','0',
        'P','1','1','1',
        'P','0','1','0',
        'R','F','1','1',
        'R','R','1','0',
        'L','F','1','1',
        'L','R','1','1',
        'L','G','1','0',
        'G','O','1','1',
        'R','0','1','0',
        'R','1','1','1',
        'T','D','1','0',
        'D','S','1','1',
        'E','S','1','0',
        'D','C','1','1',
        'E','R','0','0',
    };

    char index = 0;
    if(chars[0] == 'H' && chars[1] == 'H')
        index = HH;
    else if(chars[0] == 'P' && chars[1] == '1')
        index = P1;
    else if(chars[0] == 'P' && chars[1] == '0')
            index = P0;
    else if(chars[0] == 'R' && chars[1] == 'F')
            index = RF;
    else if(chars[0] == 'R' && chars[1] == 'R')
            index = RR;
    else if(chars[0] == 'L' && chars[1] == 'F')
            index = LF;
    else if(chars[0] == 'L' && chars[1] == 'R')
            index = LR;
    else if(chars[0] == 'G' && chars[1] == 'O')
            index = GO;
    else if(chars[0] == 'R' && chars[1] == '0')
            index = R0;
    else if(chars[0] == 'R' && chars[1] == '1')
            index = R1;
    else if(chars[0] == 'T' && chars[1] == 'D')
            index = TD;
    else if(chars[0] == 'D' && chars[1] == 'S')
            index = DS;
    else if(chars[0] == 'E' && chars[1] == 'S')
            index = ES;
    else if(chars[0] == 'D' && chars[1] == 'C')
            index = DC;
    else
        index = ER;
    char i = 0;
    UARTCharPut(UART0_BASE, 0x3A);
    for(i=0; i < 4; i++){
        UARTCharPut(UART0_BASE, LookupTable[index+i]);
    }
    UARTCharPut(UART0_BASE, 0x0D);
    UARTCharPut(UART0_BASE, 0x0A);
    return index;
}



void UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t character;
    uint32_t characters[2];
    char function=0;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        character = UARTCharGetNonBlocking(UART0_BASE);
        UARTCharPutNonBlocking(UART0_BASE, character); //echo character
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
         //turn off LED
        characters[CharacterCount] = character;
        CharacterCount++;
        if(CharacterCount == 2){
            CharacterCount =0;
            UARTCharPut(UART0_BASE, '\r');
            UARTCharPut(UART0_BASE, '\n');
            function = responseGet(characters);
            getFunction(function);
        }

        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
}

void getFunction(char function){
    if(function == R0)
        frontDistance();

}

void frontDistance(void){
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32DistAvg;
    volatile uint32_t ui32temp;
    uint8_t ui8outChar[5];
    int i = 0;
    int decimalNum = 0;

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
     SysCtlDelay(6700000);      // creates ~500ms delay - TivaWare fxn

}

int main(void) {

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    ADCinit();
    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable the UART interrupt
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts

    UARTCharPut(UART0_BASE, 'E');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'r');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'T');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, 'x');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, ':');
    UARTCharPut(UART0_BASE, ' ');

    while (1)
        ;
}
