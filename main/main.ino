#include "main.h"

#include <BLEDevice.h>
#include <sstream>
#include <iomanip>
#include "WiFiManager.h"

#include "config.h"

#include "firmwarever.h"
#include <EEPROM.h>
#define CONFIG_ESP32_DEBUG_OCDAWARE 1

#include "esp_system.h"
#include "DebugPrint.h"

#if ENABLE_OTA_WEBSERVER
#include "OTAWebServer.h"
#endif

#include "settings.h"
#include "watchdog.h"

#include "SPIFFSLogger.h"
#include "utility.h"

#if USE_MQTT
#include "mqtt_client.h"
#endif

#if USE_FHEM_LEPRESENCE_SERVER
#include "fhem_lepresence_server.h"
#endif

#include "NTPTime.h"

#include "battery_level.h"

#include"ble_manager.h"


#define SYS_INFORMATION_DELAY 120 /*2 minutes*/
unsigned long lastSySInfoTime = 0;

#if ENABLE_OTA_WEBSERVER
OTAWebServer *pwebserver;
#endif

//#ifndef LED_BUILTIN
#define LED_BUILTIN GPIO_NUM_13
//#endif

unsigned long EXt_mem =0;


///////////////////////////////////////////////////////////////////////////
//   BLUETOOTH
///////////////////////////////////////////////////////////////////////////

   ble_manager *myBLE_manager;


#if PUBLISH_BATTERY_LEVEL


#endif


///////////////////////////////////////////////////////////////////////////
//   SETUP() & LOOP()
///////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      setup
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
#if defined(DEBUG_SERIAL)
  Serial.begin(115200);
#endif
 
  DEBUG_PRINTF("start BLE tracker %s\n", Firmware::FullVersion().c_str());
  
  EXt_mem= xPortGetFreeHeapSize()-esp_get_free_internal_heap_size();

  DEBUG_PRINTF("*** SETUP begin Memory available : %u internal %u\n",xPortGetFreeHeapSize(),esp_get_free_internal_heap_size());

  LogResetReason();
  
  pinMode(LED_BUILTIN, OUTPUT);

#if defined(_SPIFFS_H_)
  if (!SPIFFS.begin())
  {
    if (!SPIFFS.format())
      DEBUG_PRINTLN("Failed to initialize SPIFFS");
  }
#endif
  //DEBUG_PRINTF("*** SETUP after SPIFFS Memory available : %u battery %.2f \n",xPortGetFreeHeapSize(),battery_level::battery_voltage());

#if ERASE_DATA_AFTER_FLASH
  int dataErased = 0; // 0 -> Data erased not performed
  String ver = Firmware::FullVersion();
  if (ver != Firmware::readVersion())
  {
    dataErased++; // 1 -> Data should be erased
    if (SPIFFS.format())
      dataErased++; // 2 -> Data erased
    Firmware::writeVersion();
  }
#endif // ERASE_DATA_AFTER_FLASH

  const char *settingsFile = "/settings.bin";
  SettingsMngr.SettingsFile(settingsFile);
  if (SPIFFS.exists(settingsFile))
    SettingsMngr.Load();


#if ENABLE_FILE_LOG
  SPIFFSLogger.Initialize("/logs.bin", MAX_NUM_OF_SAVED_LOGS);
  SPIFFSLogger.setLogLevel(SPIFFSLoggerClass::LogLevel(SettingsMngr.logLevel));
#endif
//DEBUG_PRINTF("*** SETUP after SPIFFS 2 Memory available : %u\n",xPortGetFreeHeapSize());

  if (SettingsMngr.wifiSSID.isEmpty())
  {
    StartAccessPointMode();
  }
  else
  {
    WiFiConnect(SettingsMngr.wifiSSID, SettingsMngr.wifiPwd);
  }
//DEBUG_PRINTF("*** SETUP after WIFI Memory available : %u\n",xPortGetFreeHeapSize());

#if ERASE_DATA_AFTER_FLASH
  if (dataErased == 1)
    LOG_TO_FILE_E("Error Erasing all persitent data!");
  else if (dataErased == 2)
    LOG_TO_FILE_I("Erased all persitent data!");
#endif


 myBLE_manager= new ble_manager(SettingsMngr.GetMaxNumOfTraceableDevices());

//DEBUG_PRINTF("*** SETUP after ble create : %u\n",xPortGetFreeHeapSize());

  for (const auto &dev : SettingsMngr.GetKnownDevicesList())
  {
    BLETrackedDevice trackedDevice;
    memcpy(trackedDevice.address, dev.address, ADDRESS_STRING_SIZE);
    trackedDevice.forceBatteryRead = dev.readBattery;
    myBLE_manager->addKnownDevice(trackedDevice);
   
  }
//DEBUG_PRINTF("*** SETUP after reserve Memory available : %u\n",xPortGetFreeHeapSize());

#if ENABLE_OTA_WEBSERVER
  pwebserver= new OTAWebServer(myBLE_manager);
  pwebserver->setup(SettingsMngr.gateway, SettingsMngr.wifiSSID, SettingsMngr.wifiPwd);
  pwebserver->begin();
#endif
DEBUG_PRINTF("*** SETUP after WEBSERVER Memory available : %u\n",xPortGetFreeHeapSize());

  Watchdog::Initialize();

 
  myBLE_manager->init(SettingsMngr.gateway.c_str());

  DEBUG_PRINTF("*** SETUP after BT Memory available : %u\n",xPortGetFreeHeapSize());

 if (!IsAccessPointModeOn())
  {
#if USE_MQTT
    initializeMQTT();
    connectToMQTT();
#endif

#if USE_FHEM_LEPRESENCE_SERVER
    FHEMLePresenceServer::initializeServer();
#endif
  }

  LOG_TO_FILE("BLETracker initialized");
}

char *formatMillis(unsigned long milliseconds, char outstr[20])
{
  unsigned long seconds = milliseconds / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  snprintf(outstr, 20, "%d.%02d:%02d:%02d", days, hours % 24, minutes % 60, seconds % 60);
  return outstr;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      LOOP
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  try
  {
#if USE_MQTT
    if (!IsAccessPointModeOn())
    {
      mqttLoop();
    }
#endif

#if USE_FHEM_LEPRESENCE_SERVER
    // Check and restore the wifi connection if it's loose
    if (!SettingsMngr.wifiSSID.isEmpty())
    {
      WiFiConnect(SettingsMngr.wifiSSID, SettingsMngr.wifiPwd);
    }
#endif

    Watchdog::Feed();
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    
    DEBUG_PRINTF("Main loop Free heap: %u, int %u, ext used %u battery %.2f  uptime %d \n", xPortGetFreeHeapSize() ,esp_get_free_internal_heap_size() , (EXt_mem+esp_get_free_internal_heap_size())-xPortGetFreeHeapSize(),
        battery_level::battery_voltage(), esp_timer_get_time()/1000000);
    
    if (myBLE_manager->NumDevicesTracked() == SettingsMngr.GetMaxNumOfTraceableDevices())
    {
      DEBUG_PRINTLN("INFO: Restart because the array is full\n");
      LOG_TO_FILE("Restarting: reached the max number of traceable devices");
      esp_restart();
    }

    if (!IsAccessPointModeOn() || true)  //  WTF
    {
      bool scanEnabled = SettingsMngr.IsManualScanOn() || !SettingsMngr.IsManualScanEnabled();

      if (scanEnabled)
      {
        myBLE_manager->scan();
      }
      else
      {
        myBLE_manager->clear();
      }

#if USE_MQTT
      publishAvailabilityToMQTT();
#endif

myBLE_manager->cleanList(SettingsMngr.maxNotAdvPeriod);

digitalWrite(LED_BUILTIN, LOW);  // turn the LED off

#if PUBLISH_BATTERY_LEVEL
#if PROGRESSIVE_SCAN
      if (scanCompleted)
#endif
  myBLE_manager->getBatteryLevel();
        // batteryTask();
#endif

#if USE_MQTT
      publishAvailabilityToMQTT();

      bool publishSystemInfo = ((lastSySInfoTime + SYS_INFORMATION_DELAY) < NTPTime::seconds()) || (lastSySInfoTime == 0);

      if (scanEnabled || publishSystemInfo)
      {
        for (auto &trackedDevice : BLETrackedDevices)
        {
          if (trackedDevice.isDiscovered)
          {
            publishBLEState(trackedDevice.address, MQTT_PAYLOAD_ON, trackedDevice.rssiValue, trackedDevice.batteryLevel);
          }
          else
          {
            publishBLEState(trackedDevice.address, MQTT_PAYLOAD_OFF, -100, trackedDevice.batteryLevel);
          }
        }
      }

      // System Information
      if (publishSystemInfo)
      {
        publishSySInfo();
        lastSySInfoTime = NTPTime::seconds();
      }

      publishAvailabilityToMQTT();

#elif USE_FHEM_LEPRESENCE_SERVER
      FHEMLePresenceServer::loop(); // Handle clients connections
#endif
    }
  }
  catch (std::exception &e)
  {
    DEBUG_PRINTF("Error Caught Exception %s", e.what());
    LOG_TO_FILE_E("Error Caught Exception %s", e.what());
  }
  catch (...)
  {
    DEBUG_PRINTLN("Error Unhandled exception trapped in main loop");
    LOG_TO_FILE_E("Error Unhandled exception trapped in main loop");
  }

  delay(IsAccessPointModeOn() ? 5000 : 100);
}
