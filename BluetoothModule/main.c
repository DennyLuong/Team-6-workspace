#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"


void main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

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
    {
        if (UARTCharsAvail(UART1_BASE)) {
            UARTCharPut(UART1_BASE, UARTCharGet(UART1_BASE));
            UARTCharPut(UART1_BASE, 'T');
        }
    }

}
