#ifndef INTERPRETER_H
#define INTERPRETER_H

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "RobControl.h"
#include "Misc.h"

/* Declaration of interpreter functions */

char *my_strcasestr(const char *arg1, const char *arg2);

/* converts a string block into a motion package */
unsigned short Interpreter(char* Block, MotionPackage_Type* Package);

#endif


