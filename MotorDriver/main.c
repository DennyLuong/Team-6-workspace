#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"

#define PWM_FREQUENCY 55


unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;


void PWMinit(void)
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
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}



void PWMmove(void){
	// backward
	//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 12);
	ui8Adjust = ui32Load;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 12);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust);



	// forward

	//ROM_SysCtlDelay(100000);



}



int main(void)
{
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	PWMinit();



	while(1)
	{
		PWMmove();

		ROM_SysCtlDelay(1000000);
	}



}
