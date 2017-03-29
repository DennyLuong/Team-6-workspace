/*
 * CommandInterpreter.c
 *
 *  Created on: Mar 28, 2017
 *      Author: Sikender
 */
#include "CommandInterpreter.h"

typedef enum{
    HH,
    P1,
    P0,
    MF,
    RR,
    SS,
    LR,
    LG,
    GO,
    R0,
    R1,
    TD,
    DS,
    ES,
    DC,
    ER
}Commands;

static const char * LookupTable[] = {
            ":HH Enter 2char function\n",
            ":P1 PWM enable\n",
            ":P0 PWM disable\n",
            ":MF Moving Forward\n",
            ":RR Right Wheel Reverse\n",
            ":SS Set Speed\n",
            ":LR Left Wheel Reverse\n",
            ":LG Light Get\n",
            ":GO PID initiated\n",
            ":R0 Read Front Distance Sensor\n",
            ":R1 Read Right Distance Sensor\n",
            ":TD Toggle Data Acquisition\n",
            ":DS Post Drive Semaphore\n",
            ":ES Emergency Stop\n",
            ":DC Drive Clock Start\n",
            ":ER Error\n"
    };

char * responseGet(char chars[]){
    char index = 0;
    if(!strcmp(chars,"HH"))
        index = HH;
    else if(!strcmp(chars,"P1"))
        index = P1;
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
    else
        index = ER;

    command = index;
    return LookupTable[index];
}

void callFunction(void){
    switch(command){
    case HH:
        break;
    default:
        break;
    }
}



