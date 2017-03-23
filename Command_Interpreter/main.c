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
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/debug.h"

#define PWM_FREQUENCY 55

typedef enum{
    HH=0,
    P1=2,
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

char responseGet(uint32_t chars[]){
    static const char * LookupTable[] = {
            "HH", " Enter 2char function      ",
            "P1", " PWM enable                ",
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
    if(chars[0] == 'H' && chars[1] == 'H')
        index = HH;
    else if(chars[0] == 'P' && chars[1] == '1')
        index = P1;
    else if(chars[0] == 'P' && chars[1] == '0')
            index = P0;
    else if(chars[0] == 'M' && chars[1] == 'F')
            index = MF;
    else if(chars[0] == 'R' && chars[1] == 'R')
            index = RR;
    else if(chars[0] == 'S' && chars[1] == 'S')
            index = SS;
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

    UARTCharPut(UART1_BASE, 0x3A);
    UARTCharPut(UART1_BASE, 0x20);
    print(LookupTable[index], 2);
    UARTCharPut(UART1_BASE, 0x20);
    print(LookupTable[index+1], 27);
    UARTCharPut(UART1_BASE, 0x20);
    UARTCharPut(UART1_BASE, 0x0D);
    UARTCharPut(UART1_BASE, 0x0A);
    return index;
}



void UARTIntHandler(void)
{
     uint32_t ui32Status;
     uint32_t character;
     uint32_t characters[2];
     char function=0;

     ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status
     UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts

     while(UARTCharsAvail(UART1_BASE)) //loop while there are chars
     {
         character = UARTCharGetNonBlocking(UART1_BASE);
         UARTCharPutNonBlocking(UART1_BASE, character); //echo character
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
          //turn off LED
         characters[CharacterCount] = character;
         CharacterCount++;
         if(CharacterCount == 2){
             CharacterCount =0;
             UARTCharPut(UART1_BASE, '\r');
             UARTCharPut(UART1_BASE, '\n');
             function = responseGet(characters);
             getFunction(function);
             print("Enter Command:", 14);

         }

         SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
}

void getFunction(char function){
    switch (function) {
    case R0:
        frontDistance();
        break;
    case MF:
    	PWMForward();
    	break;
    case ES:
    	PWMStop();
    case SS:
    	PWMSpeed();
    default:
		print("error",5);
    }


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
    //UARTStdioConfig(UART1_BASE, 115200, SysCtlClockGet());

    while(!ADCIntStatus(ADC0_BASE, 1, false))
    {
    }
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    ui32DistAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;



    // FIX: output avg distance to terminal
    // FIX: fix range value for inches or centimeters (OPTIONAL)
    print("Distance: ");

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
        UARTCharPut(UART1_BASE, ui8outChar[i]);
    }

    UARTCharPut(UART1_BASE, '\r');
    UARTCharPut(UART1_BASE, '\n');
    SysCtlDelay(6700000);
}

void print(const char * input, uint32_t len){
    uint32_t count;
    for(count =0 ; count< len; count++){
        UARTCharPut(UART1_BASE, input[count]);
    }
}

void UARTInit(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void ADCInit(void) {
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


unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;


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
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void PWMForward(void){
	// forward
	ui8Adjust = ui32Load;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 12);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8Adjust);
}

void PWMStop(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), false);
}

void PWMSpeed(void){
	print("Enter Speed (0-9",3);
	ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status
	     UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts

	     while(UARTCharsAvail(UART1_BASE)) //loop while there are chars
	     {
	         character = UARTCharGetNonBlocking(UART1_BASE);
	         UARTCharPutNonBlocking(UART1_BASE, character);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speed * ui8Adjust/100);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speed * ui8Adjust/100);
}


int main(void) {
    UARTInit();
    ADCInit();
    PWMInit();

    IntMasterEnable();
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    print("Embedded Systems - TEAM 6 \r\n",29);
    print("Enter Command:", 14);

    while (1)
        ;
}
