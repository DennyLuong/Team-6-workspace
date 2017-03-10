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

struct DataFrame{
    uint8_t Start;
    uint8_t Function;
    uint8_t Data;
    uint8_t CheckSum;
    uint8_t CR;
    uint8_t LF;
};

typedef enum {
    HH = 0,
    P1 = 3,
    P0 = 6,
    RF = 9,
    RR = 12,
    LF = 15,
    LR = 18,
    LG = 21,
    GO = 24,
    R0 = 27,
    R1 = 30,
    TD = 33,
    DS = 36,
    ES = 39,
    DC = 42
}Commands;

static const char LookupTable[] = {
    HH,0,0,
    P1,1,1,
    P0,1,0,
    RF,1,1,
    RR,1,0,
    LF,1,1,
    LG,1,0,
    GO,1,1,
    R0,1,0,
    R1,1,1,
    TD,1,0,
    DS,1,1,
    ES,1,0,
    DC,1,1,
};

uint8_t CharacterCount = 0;

//struct Dataframe responseGet(uint32_t chars[]){
//    struct DataFrame response;
//    uint8_t index = (chars[0] == 'H' && chars[1] == 'H')? HH :
//                    (chars[0] == 'P' && chars[1] == '1')? P1 :
//                    (chars[0] == 'P' && chars[1] == '0')? HH :
//                    (chars[0] == 'R' && chars[1] == 'F')? P1 :
//                    (chars[0] == 'R' && chars[1] == 'R')? HH :
//                    (chars[0] == 'L' && chars[1] == 'F')? P1 : DC;
//    response.Start = 0x3A;
//    response.Function = LookupTable[index];
//    response.Data = LookupTable[index+1];
//    response.CheckSum = LookupTable[index+2];
//    response.CR = 0x0D;
//    response.LF = 0x0A;
//
//    return response;
//}



void UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint32_t character;
    uint32_t characters[2];

    struct DataFrame responseData;

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
        UARTCharPut(UART0_BASE, characters[0]);
        if(CharacterCount == 2){
            CharacterCount =0;
            UARTCharPut(UART0_BASE, '\r');
            UARTCharPut(UART0_BASE, '\n');
//            responseData = responseGet(characters);
//            UARTCharPut(UART0_BASE, responseData.Start);
//            UARTCharPut(UART0_BASE, responseData.Function);
//            UARTCharPut(UART0_BASE, responseData.Data);
//            UARTCharPut(UART0_BASE, responseData.CheckSum);
//            UARTCharPut(UART0_BASE, responseData.CR);
//            UARTCharPut(UART0_BASE, responseData.LF);//
        }

        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
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
