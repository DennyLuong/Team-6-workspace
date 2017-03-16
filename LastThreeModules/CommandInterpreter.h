#ifdef COMMANDINTERPRETER_H
#define COMMANDINTERPRETER_H

#include <stdint.h>
#include <stdbool.h>

extern char CI_get_response(char command);
extern char CI_set_command(char command[]);
extern void CI_call_function(char command);

#endif
