#include "utility.h"

#include "SPIFFSLogger.h"
#include "DebugPrint.h"
#include <stdint.h>


void vApplicationMallocFailedHook(void)
{
  DEBUG_PRINTLN("---MallocFailed----");
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  DEBUG_PRINTF("StackOverflow:%x (%s)\n", xTask, pcTaskName);
}


//Convert mac adddress from ABCEF1234567 to AB:CE:F1:23:45:67 or ab:ce:f1:23:45:67
void CanonicalAddress(const char address[ADDRESS_STRING_SIZE], char out[ADDRESS_STRING_SIZE + 5] , bool upperCase)
{
  char *outWlkr = out;
  for (int i = 0; i < ADDRESS_STRING_SIZE; i++)
  {
    *outWlkr = upperCase ? toupper(address[i]) : tolower(address[i]);
    outWlkr++;
    if ((i & 1) == 1 && i != (ADDRESS_STRING_SIZE - 2))
    {
      *outWlkr = ':';
      outWlkr++;
    }
  }
}
//Convert mac adddress to ABCEF1234567
void NormalizeAddress(const uint8_t address[6], char out[ADDRESS_STRING_SIZE])
{
  snprintf(out, ADDRESS_STRING_SIZE,"%02X%02X%02X%02X%02X%02X", address[0], address[1], address[2], address[3], address[4], address[5]);
}

//Convert mac adddress from AB:CE:F1:23:45:67 or ab:ce:f1:23:45:67 to ABCEF1234567
void NormalizeAddress(const char* in, char out[ADDRESS_STRING_SIZE])
{
    char *outWlkr = out;
    for (int i = 0; in[i]!='\0'; i++)
    {
      const char c = in[i];
      if (c == ':')
        continue;
      *outWlkr = toupper(c);
      outWlkr++;
    }
    out[ADDRESS_STRING_SIZE - 1] = '\0';
}

//Convert mac adddress from AB:CE:F1:23:45:67 or ab:ce:f1:23:45:67 to ABCEF1234567
void NormalizeAddress(const std::string &in, char out[ADDRESS_STRING_SIZE])
{
  NormalizeAddress(in.c_str(),out);
}



void LogResetReason()
{
  esp_reset_reason_t r = esp_reset_reason();
  const char *msg;
  switch (r)
  {
  case ESP_RST_POWERON:
    msg = "Reset due to power-on event";
    break;
  case ESP_RST_EXT:
    msg = "Reset by external pin";
    break;
  case ESP_RST_SW:
    msg = " Software reset via esp_restart";
    break;
  case ESP_RST_PANIC:
    msg = "Software reset due to exception/panic";
    break;
  case ESP_RST_INT_WDT:
    msg = "Reset (software or hardware) due to interrupt watchdog";
    break;
  case ESP_RST_TASK_WDT:
    msg = "Reset due to task watchdog";
    break;
  case ESP_RST_WDT:
    msg = "Reset due to other watchdogs";
    break;
  case ESP_RST_DEEPSLEEP:
    msg = "Reset after exiting deep sleep mode";
    break;
  case ESP_RST_BROWNOUT:
    msg = "Brownout reset (software or hardware)";
    break;
  case ESP_RST_SDIO:
    msg = "Reset over SDIO";
    break;
  case ESP_RST_UNKNOWN:
  default:
    msg = "Reset reason can not be determined";
    break;
  }
  DEBUG_PRINTLN(msg);
  LOG_TO_FILE_E(msg);
}