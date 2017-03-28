#include "CommandInterpreter.h"
#include "DistanceSensor.h"


typedef enum{
    HH=0, 
    P1=2, 
    P0=4, 
    RF=6,
    RR=8,
    LF=10,
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

char CI_get_response(char command){
    static const char * LookupTable[] = {
    	"HH", "Help",
	    "P1", "PWM enable",
	    "P0", "PWM disable",
	    "RF", "Right Wheel Forward",
	    "RR", "Right Wheel Reverse",
	    "LF", "Left Wheel Forward",
	    "LR", "Left Wheel Reverse", 
	    "LG", "Light Get", 
	    "GO", "PID initiated", 
	    "R0", "Read Front Distance Sensor",
	    "R1", "Read Right Distance Sensor",
	    "TD", "Toggle Data Acquisition",
	    "DS", "Post Drive Semaphore", 
	    "ES", "Emergency Stop",
	    "DC", "Drive Clock Start",
    	"ER", "Error"
    };

    //
    // print start 0x3A
   // LookupTable[command];
 //   LookupTable[command+1];
    // print check sum --
    // print 0x0D
    // print 0x0A

    return (char) command; 
}

char CI_set_command(char command[]){
	if(command[0] == 'H' && command[1] == 'H')
        return HH;
    else if(command[0] == 'P' && command[1] == '1')
        return P1;
    else if(command[0] == 'P' && command[1] == '0')
        return P0;
    else if(command[0] == 'R' && command[1] == 'F')
        return RF;
    else if(command[0] == 'R' && command[1] == 'R')
        return RR;
    else if(command[0] == 'L' && command[1] == 'F')
        return LF;
    else if(command[0] == 'L' && command[1] == 'R')
        return LR;
    else if(command[0] == 'G' && command[1] == 'O')
        return GO;
    else if(command[0] == 'R' && command[1] == '0')
        return R0;
    else if(command[0] == 'R' && command[1] == '1')
        return R1;
    else if(command[0] == 'T' && command[1] == 'D')
        return TD;
    else if(command[0] == 'D' && command[1] == 'S')
        return DS;
    else if(command[0] == 'E' && command[1] == 'S')
        return ES;
    else if(command[0] == 'D' && command[1] == 'C')
        return DC;
    else
        return ER;
}

void CI_call_function(char command){
	switch(command){
		case HH:
			break;
		case P1:
			break;
		case P0:
			break; 
		case RF:
			break;
		case RR:
			break; 
		case LF:
			break;
		case LR:
			break;
		case LG:
			break;
		case GO:
			break;
		case R0:
			get_distance(0); 
			break;
		case R1:
			get_distance(1); 
			break;
		case TD:
			break;
		case DS:
			break;
		case ES:
			break;
		case DC:
			break;
		default: 

	}
}
