/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* TI Tiva C Header files*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

/* Board Header file */
#include "Board.h"

#define TASKSTACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
void heartBeatFxn(UArg arg0, UArg arg1)
{
    while (1) {
        Task_sleep((UInt)arg0);
        GPIO_toggle(Board_LED0);
    }
}



void UART_Callbk_read(UART_Handle handle, void * ptr, size_t size){
    UART_read(handle, ptr, size); 
    UART_write(handle, (char) ptr, size);
}

void UART_Callbk_write(UART_Handle hangle, void * ptr, size_t size){
    UART_write(hangle, (char) ptr, size);
}

/*
 *  ======== MAIN ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    IntMasterEnable();
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();   // need to change EK_TM4C123GXL_initUART for bluetooth
                        // calls UART_init()
   
    // Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);


    UART_Handle uart;
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.dataLength = UART_LEN_8;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.baudRate = 9600;

    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &UART_Callbk_read;
    uartParams.writeCallback = &UART_Callbk_write;
    uart = UART_open(Board_UART0, &uartParams);


    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
