#ifndef MAIN_H
#define MAIN_H
#include <WString.h>
#include <sstream>
#include <iomanip>
#include "config.h"
#include "firmwarever.h"

#define DESCRIPTION_STRING_SIZE 21

char *formatMillis(unsigned long milliseconds, char outStr[20]);
void ForceBatteryRead(const char *normalizedmac);

#endif /*MAIN_H*/
