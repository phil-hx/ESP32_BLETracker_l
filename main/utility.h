#ifndef __UTILITY_H__
#define __UTILITY_H__
#include "main.h"
#include <BLEAddress.h>
#include "firmwarever.h"
//#include <task_snapshot.h>

#define ADDRESS_STRING_SIZE 13
#define MAX_Dname 8

/*
extern "C"
{
  void vApplicationMallocFailedHook(void);
  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
}
*/


//Convert mac adddress from ABCEF1234567 to AB:CE:F1:23:45:67 or ab:ce:f1:23:45:67
void CanonicalAddress(const char address[ADDRESS_STRING_SIZE], char out[ADDRESS_STRING_SIZE + 5] , bool upperCase = true);
void NormalizeAddress(const std::string &in, char out[ADDRESS_STRING_SIZE]);
void NormalizeAddress(const char* in, char out[ADDRESS_STRING_SIZE]);
void NormalizeAddress(const uint8_t address[6], char out[ADDRESS_STRING_SIZE]);

void LogResetReason();

#endif /*__UTILITY_H__*/