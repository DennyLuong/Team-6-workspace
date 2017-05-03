/*
 * [ECE XX37] - Embedded Systems
 * Spring 2017
 *
 * Team 6
 * Authors: Robert Duenez, Katherine Perez, Sikender Shahid
 *
 * Milestone 10: Rewriting of main in RTOS
 *
 */


//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h>


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"
#include "string.h"

/********************************************************************************************************
 *                                     ADC get_adc_value Functions                                      *
 ********************************************************************************************************/
uint32_t rightDistance(void){
    //ADC_CTL_CH0
	uint32_t ui32ADC0Value[4];
	ADCIntClear(ADC0_BASE, 1);
	ADCProcessorTrigger(ADC0_BASE, 1);
	while(!ADCIntStatus(ADC0_BASE, 1, false)){}
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    return ui32ADC0Value[0];
}
uint32_t frontDistance(void){
    //ADC_CTL_CH1
	uint32_t ui32ADC0Value[4];
	ADCIntClear(ADC0_BASE, 2);
	ADCProcessorTrigger(ADC0_BASE, 2);
	while(!ADCIntStatus(ADC0_BASE, 2, false)){}
	ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);
    return ui32ADC0Value[0];
}
/********************************************************************************************************
 *                                             PWM Functions                                            *
 ********************************************************************************************************/
#define PWM_FREQUENCY 10000

unsigned int ui32Load = 0;
unsigned int ui32PWMClock = 0;
unsigned int ui8Adjust = 83;

void move_forward(void){
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_2|GPIO_PIN_7);
}

void move_reverse(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, 0|0);
}

void move_cw(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_2|0);
}

void move_ccw(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_7|0);
}

void PWMStop(void){
	PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), false);
}

void PWMSetSpeed(unsigned int left_adjust, unsigned int right_adjust){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, left_adjust * ui32Load/100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, right_adjust * ui32Load/100);
}

void PWMStart(void){
    PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT|PWM_OUT_1_BIT), true);
    PWMSetSpeed(ui32Load, ui32Load);
}

void PWMUserSetSpeed(void){
	unsigned char inSpeed[3];
	UARTprintf("Enter Speed (1-100): ");
	UARTgets(inSpeed, 4);
	PWMSetSpeed(atoi(inSpeed), atoi(inSpeed));
}
/********************************************************************************************************
 *                                          Double Buffer                                               *
 ********************************************************************************************************/
void swap(char * buffer1 , char * buffer2 ){
    char * temp = buffer1;
    buffer1 = buffer2;
    buffer2 = temp;
}

char distanceBuffer[20];
char distanceBuffer2[20];
int distanceBufferCount = 0;
void distanceBufferLog(int ErrorValue){
    if(distanceBufferCount != 20){
        distanceBuffer[distanceBufferCount] = ErrorValue;
        distanceBufferCount++;
    }
    else{
        for(distanceBufferCount = 0 ; distanceBufferCount <20 ; distanceBufferCount++){
            UARTprintf("%d", distanceBuffer[distanceBufferCount]);
            UARTprintf("\n");
        }
        swap(distanceBuffer, distanceBuffer2);
        distanceBufferCount = 0;
    }
}

/********************************************************************************************************
 *                                       PID Control Functions                                          *
 ********************************************************************************************************/
uint32_t time = 0;
uint32_t previousTimeRead = 0;
uint32_t initialTime, initialLine, finalLine, thresholdBlackLine;
int blackLineWidth = 0;
bool blackLineFound(void)
{
    TimerDisable(TIMER2_BASE, TIMER_A);
    SysCtlDelay(100);
    TimerEnable(TIMER0_BASE,TIMER_BOTH);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 );
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4  , GPIO_PIN_4  );
    SysCtlDelay(100);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4  );
    initialTime = TimerValueGet(TIMER0_BASE, TIMER_A);
    while(GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_4)){};
    time = TimerValueGet(TIMER0_BASE, TIMER_A) - initialTime;

    //white noise handle
    if(time < 400){
        time = 0;
    }
    //first detection of black line
    if(time - previousTimeRead > 0 && initialLine == 0)
        initialLine = time;

    //detection of black line decreasing
    if(time - previousTimeRead < 0)
        finalLine = time;

    //thresholding value
    thresholdBlackLine = finalLine - initialLine;
    //threshold action
    if(thresholdBlackLine!= 0){
            blackLineWidth++;
            UARTprintf("thresholding black line value : %u\n",thresholdBlackLine);
        }
    if(thresholdBlackLine == 0){
        blackLineWidth = 0;
    }
    if (blackLineWidth >5){
        PWMStop();
    }
    if (blackLineWidth > 2){
        UARTprintf("Passed blackLine\n");
    }
    // clearing out values after black line detection
    previousTimeRead = time;
    if(previousTimeRead == 0 && time == 0){
        initialLine = 0;
        finalLine = 0;
        thresholdBlackLine = 0;
        blackLineWidth = 0;
    }
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    SysCtlDelay(100);
    TimerEnable(TIMER2_BASE, TIMER_A);
    return false;
}

void PIDStart(void) {
	// enable Timer2, SubtimerA
	TimerEnable(TIMER2_BASE, TIMER_A);
	unsigned int speed = 2840;
	PWMSetSpeed(speed, speed);
	PWMStart();
	move_forward();
}

const uint32_t front_target = 2200;
uint32_t cur_pos;
const uint32_t target_dist = 2000;
int proportional =0;
int last_proportional = 0;
int derivative = 0;
int integral = 0;
const float p_const = 10;
const float i_const =  3500;
const float d_const = .0075;
const int max = 100;
int dataCollectionToggle = 0;
int power_difference = 0;
bool lineDetectToggle= 0;

int computePID(void){
	if(blackLineFound()){
		lineDetectToggle = !(lineDetectToggle);
	}

	// PID calculations
	cur_pos = rightDistance();
	proportional = cur_pos - target_dist;
	derivative = proportional - last_proportional;
	integral += proportional;
	last_proportional = proportional;

	power_difference = proportional/p_const + derivative * d_const + integral/i_const;

	if(lineDetectToggle && dataCollectionToggle){
	    distanceBufferLog(power_difference);
	}
	dataCollectionToggle = 1 - dataCollectionToggle; // or dataCollectionToggle = !dataCollectionToggle;

	if (power_difference > max){
		power_difference = max;}
	if (power_difference < -max){
		power_difference = -max;}

//	UARTprintf("Power Difference: %d\n",power_difference);

	  if(power_difference < 0){
		  PWMSetSpeed(max, 101 + power_difference);
	  }
	  else if(power_difference > 0){
		  PWMSetSpeed(max + 1 -power_difference, max);
	  }
	  else{
		  PWMSetSpeed(max, max);
	  }
	  //front_target = 2200
	  if(frontDistance() > front_target){
		  while (!(frontDistance() < 1500))
	  			move_cw();
		  move_forward();
	  }

	  if(rightDistance() < 800 )
	  {
		  while(!(rightDistance() > 2400))
	  		{
	  			move_forward();
	  			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Load);
	  			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Load/2);
	  		}
	  	}
	return power_difference;
}

/********************************************************************************************************
 *                                     Timer2 Interrupt Handler                                         *
 ********************************************************************************************************/

void Timer2IntHandler(void) {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	Swi_post(PIDswi);
}

/********************************************************************************************************
 *                                     UART Handler and Command Functions                               *
 ********************************************************************************************************/


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
    ER=30,
    BL=32
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
            "ER", " Error                     ",
            "BL", " Black Line                ",
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
    else if(!strcmp(chars,"LG"))
    	index = LG;
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
    else if(!strcmp(chars,"BL"))
        index = BL;
    else
        index = ER;

    UARTprintf("\n");
    UARTprintf(LookupTable[index]);
    UARTCharPut(UART1_BASE, 0x20);
    UARTprintf(LookupTable[index + 1]);
    UARTprintf(" \r\n");
    return index;
}

void getFunction(char function){
    switch (function) {
    case R1:
        rightDistance();
        frontDistance();
        break;
    case MF:
    	move_forward();
        break;
    case RR:
    	move_cw();
        break;
    case LR:
    	move_ccw();
        break;
    case ES:
        PWMStop();
        break;
    case SS:
    	PWMUserSetSpeed();
        break;
    case MS:
        PWMStart();
        break;
    case GO:
        PIDStart();
        break;
    case BL:
        blackLineFound();
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
    //get interrupt status
    ui32Status = UARTIntStatus(UART1_BASE, true);
    //clear the asserted interrupts
    UARTIntClear(UART1_BASE, ui32Status);
    UARTgets(characters, 3);
    function = responseGet(characters);
    getFunction(function);
    UARTprintf("Enter Command: ");
}
/********************************************************************************************************
 *                                     Interface Initialization Functions                               *
 ********************************************************************************************************/

void UARTInit(void){
    // UART Module 1
    // Port PB0 RX
    // Port PB1 TX

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// configuration of GPIO pins and types
	GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// set clock and standard I/O
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, 115200, 16000000);
	// Interrupt Enabled for Data on TX and RX
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void ADCInit(void) {
    //  Port PE3 AIN-0 - ADC_CTL_CH0 - RIGHT DISTANCE SENSOR
    //  Port PE2 AIN-1 - ADC_CTL_CH1 - FRONT DISTANCE SENSOR
    //  Port PE5 AIN-8 - ADC_CTL_CH8 - RIGHT LIGHT SENSOR
    //  Port PE4 AIN-9 - ADC_CTL_CH9 - LEFT LIGHT SENSOR

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);

	ADCSequenceEnable(ADC0_BASE, 1);
	ADCSequenceEnable(ADC0_BASE, 2);
}

void PWMInit(void)
{
    //  Port PE1 - MODE = 1
    //  Port PD3 - PHASE PIN - RIGHT MOTOR  // changed to PD7
    //  Port PD1 - PWM PIN   - RIGHT MOTOR
    //  Port PD2 - PHASE PIN - LEFT MOTOR
    //  Port PD0 - PWM PIN   - LEFT MOTOR
    // FORWARD 0 BITMASKED  // LOGIC SWITCHED - > FORWARD 1 BITMASKED
    // REVERSE 1 BITMASKED  // LOGIC SWITCHED - > REVERSE 0 BITMASKED

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// hardware unlock for gpio pd7
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
	// enable mode = 1
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void TimerInit(void) {

	uint32_t ui32Period;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// enable timer0 as 32-bit timer in periodic mode
	TimerConfigure(TIMER2_BASE, TIMER_CFG_A_PERIODIC);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	// Interrupt at 20Hz
	ui32Period = (SysCtlClockGet() / 20);
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period-1);
	// enable timer2a interrupt vector
	IntEnable(INT_TIMER2A);
	// enable specific vector within timer to generate an interrupt
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void) {
	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 4 = 50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	UARTInit();
	ADCInit();
	PWMInit();
	TimerInit();

	UARTprintf("Embedded Systems - TEAM 6\r\n");
	UARTprintf("Type command:");

	BIOS_start();
}
