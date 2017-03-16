
#include "driverlib/adc.h"
#include "DistanceSensor.h"

uint32_t get_distance(char command){
	uint32_t ADC_value[4]; 
	volatile uint32_t average_distance; 
	volatile uint32_t temp; 
	volatile char decimal_count=0; 
	uint8_t distance[5]; 
	uint64_t ADC; 
	if(!command)
		ADC = 0x40038000; // front side sensor 
	else 
		ADC = 0x40039000; // right side sensor 

	ADCIntClear(ADC, 1);
    ADCProcessorTrigger(ADC0_BASE, 1);

    while(!ADCIntStatus()){

    }
    ADCSequenceDataGet(ADC, 1, ADC_value); 
    average_distance = (ADC_value[0] + ADC_value[1] + ADC_value[2] + ADC_value[3] +2)/4; 

    
    temp = average_distance; 
    char i =0; 
    for(i = 0; temp >=1 ; i++){
    	temp = temp/10; 
    	decimal_count++;
    }
	
	for(i = decimal_count -1 ; i>=0 ; i--){
		distance[i] = (char) 48 + (average_distance % 10);
		average_distance /= 10; 
	}

	return distance; 
}


