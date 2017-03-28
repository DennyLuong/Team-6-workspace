/*
 * CommandInterpreter.h
 *
 *  Created on: Mar 21, 2017
 *      Author: Sikender
 */

#ifndef COMMANDINTERPRETER_H_
#define COMMANDINTERPRETER_H_



#include <stdint.h>
#include <stdbool.h>

extern char CI_get_response(char command);
extern char CI_set_command(char command[]);
extern void CI_call_function(char command);

#endif /* COMMANDINTERPRETER_H_ */
