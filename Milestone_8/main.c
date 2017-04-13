#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"           //Definitions for the interrupts and registers
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"


uint32_t initialCount,
         finalCount,
         count1,
		 count2;

void ConfigureTimer (void) {
    /* Enable the clock to PortF */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* Enable the clock to timer 0 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure the timer */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);



}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    ConfigureTimer();
    TimerEnable(TIMER0_BASE, TIMER_BOTH);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);

    while(1)
    {

        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);

        SysCtlDelay(100);

        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);

        initialCount = TimerValueGet(TIMER0_BASE, TIMER_A);

        while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) != 0){};

        finalCount = TimerValueGet(TIMER0_BASE, TIMER_A);

        count1 = finalCount - initialCount;

        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);

        SysCtlDelay(100);

        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);

        initialCount = TimerValueGet(TIMER0_BASE, TIMER_A);

        while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_5) != 0){};

        finalCount = TimerValueGet(TIMER0_BASE, TIMER_A);

        count2 = finalCount - initialCount;


    }


}
