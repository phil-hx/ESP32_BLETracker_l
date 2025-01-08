#include "ble_manager.h"


#include <BLEDevice.h>
#include "config.h"
#include "watchdog.h"
#include "settings.h"
#include "watchdog.h"
#include "NTPTime.h"
#include "myRWMutex.h"
#include "SPIFFSLogger.h"
#include "DebugPrint.h"


int actual_MIN_BLE_rssiValue = MIN_BLE_rssiValue;
MyAdvertisedDeviceCallbacks::MyAdvertisedDeviceCallbacks( ble_manager *pBLE) : BLEAdvertisedDeviceCallbacks(){
  pble_manager=pBLE;
}

void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    Watchdog::Feed();
    const uint8_t shortNameSize = 31;
    char address[ADDRESS_STRING_SIZE],Caddress[ADDRESS_STRING_SIZE+5];
    NormalizeAddress(*(advertisedDevice.getAddress().getNative()), address);
    CanonicalAddress(address,Caddress );

    if (!SettingsMngr.IsTraceable(address))
      return;

    int RSSI = advertisedDevice.getRSSI();

    if (RSSI < actual_MIN_BLE_rssiValue){
      // device too far 
      // DEBUG_PRINTF("INFO: device ignored , Address: %s , RSSI: %d\n", Caddress, RSSI);
      pble_manager->too_far_devices ++;
      return;
    } 

    char shortName[shortNameSize];
    memset(shortName, 0, shortNameSize);
    if (advertisedDevice.haveName())
      strncpy(shortName, advertisedDevice.getName().c_str(), shortNameSize - 1);

    CRITICALSECTION_WRITESTART(pble_manager->trackedDevicesMutex)
    
    for (auto &trackedDevice : pble_manager->BLETrackedDevices)
    {
      if (strcmp(address, trackedDevice.address) == 0)
      {
#if NUM_OF_ADVERTISEMENT_IN_SCAN > 1
        trackedDevice.advertisementCounter++;
        // To proceed we have to find at least NUM_OF_ADVERTISEMENT_IN_SCAN duplicates during the scan
        // and the code have to be executed only once
        if (trackedDevice.advertisementCounter != NUM_OF_ADVERTISEMENT_IN_SCAN)
          return;
#endif

        if (!trackedDevice.advertised) // Skip advertised dups
        {
          trackedDevice.addressType = advertisedDevice.getAddressType();
          trackedDevice.advertised = true;
          trackedDevice.lastDiscoveryTime = NTPTime::seconds();
          trackedDevice.rssiValue = RSSI;
          if (!trackedDevice.isDiscovered)
          {
            trackedDevice.isDiscovered = true;
            trackedDevice.connectionRetry = 0;
            pble_manager->FastDiscovery[trackedDevice.address] = true;
            DEBUG_PRINTF("INFO: Tracked device discovered again, Address: %s , RSSI: %d (%s)\n", Caddress, RSSI, shortName);
           if (advertisedDevice.haveName())
            {
              LOG_TO_FILE_D("Device %s ( %s ) within range, RSSI: %d ", Caddress, shortName, RSSI);
            }
            else
              LOG_TO_FILE_D("Device %s within range, RSSI: %d ", Caddress, RSSI);
          }
          else
          {
            DEBUG_PRINTF("INFO: Tracked device discovered, Address: %s , RSSI: %d ()\n", Caddress,  RSSI, shortName);
          }
        }
        return;
      }
    }

    // This is a new device...
    BLETrackedDevice trackedDevice;
    trackedDevice.advertised = NUM_OF_ADVERTISEMENT_IN_SCAN <= 1; // Skip duplicates
    memcpy(trackedDevice.address, address, ADDRESS_STRING_SIZE);
    trackedDevice.addressType = advertisedDevice.getAddressType();
    trackedDevice.isDiscovered = NUM_OF_ADVERTISEMENT_IN_SCAN <= 1;
    trackedDevice.lastDiscoveryTime = NTPTime::seconds();
    trackedDevice.lastBattMeasureTime = 0;
    trackedDevice.batteryLevel = -1;
    trackedDevice.hasBatteryService = true;
    trackedDevice.connectionRetry = 0;
    trackedDevice.rssiValue = RSSI;
    trackedDevice.advertisementCounter = 1;
    strncpy(trackedDevice.dshortName, shortName, MAX_Dname);trackedDevice.dshortName[MAX_Dname]='\0';

    pble_manager->BLETrackedDevices.push_back(std::move(trackedDevice));
    pble_manager->FastDiscovery[trackedDevice.address] = true;
#if NUM_OF_ADVERTISEMENT_IN_SCAN > 1
    // To proceed we have to find at least NUM_OF_ADVERTISEMENT_IN_SCAN duplicates during the scan
    // and the code have to be executed only once
    return;
#endif
    CRITICALSECTION_WRITEEND;

    DEBUG_PRINTF("INFO: Device discovered, Address: %s , RSSI: %d (%s)\n", Caddress, RSSI,shortName);
 
 
    if (advertisedDevice.haveName())
      LOG_TO_FILE_D("Discovered new device  %s ( %s ) within range, RSSI: %d ", Caddress,  shortName, RSSI);
    else
      LOG_TO_FILE_D("Discovered new device %s  within range, RSSI: %d ", Caddress,  RSSI);
  }



#if PUBLISH_BATTERY_LEVEL

 static BLEUUID service_BATT_UUID(BLEUUID((uint16_t)0x180F));

 static BLEUUID char_BATT_UUID(BLEUUID((uint16_t)0x2A19));

class MyBLEClientCallBack : public BLEClientCallbacks
{
  void onConnect(BLEClient *pClient)
  {
  }

  virtual void onDisconnect(BLEClient *pClient)
  {
    log_i(" >> onDisconnect callback");
    pClient->disconnect();
  }
};

void ble_manager::ForceBatteryRead(const char *normalizedAddr)
{
  for (auto &trackedDevice : BLETrackedDevices)
  {
    if (strcmp(trackedDevice.address, normalizedAddr) == 0)
    {
      trackedDevice.forceBatteryRead = true;
      return;
    }
  }
}

bool batteryLevel(const char address[ADDRESS_STRING_SIZE], esp_ble_addr_type_t addressType, int8_t &battLevel, bool &hasBatteryService)
{
 

  log_i(">> ------------------batteryLevel----------------- ");
  bool bleconnected;
  BLEClient client;
  battLevel = -1;
  static char canonicalAddress[ADDRESS_STRING_SIZE + 5];
  CanonicalAddress(address, canonicalAddress);
  BLEAddress bleAddress = BLEAddress(canonicalAddress);
  log_i("connecting to : %s", bleAddress.toString().c_str());
  LOG_TO_FILE_D("Reading battery level for device %s", address);
  MyBLEClientCallBack callback;
  client.setClientCallbacks(&callback);

  // Connect to the remote BLE Server.
  bleconnected = client.connect(bleAddress, addressType);
  if (bleconnected)
  {
    log_i("Connected to server");
    BLERemoteService *pRemote_BATT_Service = client.getService(service_BATT_UUID);
    if (pRemote_BATT_Service == nullptr)
    {
      log_i("Cannot find the BATTERY service.");
      LOG_TO_FILE_E("Error: Cannot find the BATTERY service for the device %s", address);
      hasBatteryService = false;
    }
    else
    {
      BLERemoteCharacteristic *pRemote_BATT_Characteristic = pRemote_BATT_Service->getCharacteristic(char_BATT_UUID);
      if (pRemote_BATT_Characteristic == nullptr)
      {
        log_i("Cannot find the BATTERY characteristic.");
        LOG_TO_FILE_E("Error: Cannot find the BATTERY characteristic for device %s", address);
        hasBatteryService = false;
      }
      else
      {
        std::string value = pRemote_BATT_Characteristic->readValue();
        if (value.length() > 0)
          battLevel = (int8_t)value[0];
        log_i("Reading BATTERY level : %d", battLevel);
        LOG_TO_FILE_I("Battery level for device %s is %d", address, battLevel);
        DEBUG_PRINTF("INFO: battery level for  Address: %s , level : %d \n", address, battLevel);
        hasBatteryService = true;
      }
    }
    // Before disconnecting I need to pause the task to wait (I don't know what), otherwise we have an heap corruption
    // delay(200);
    if (client.isConnected())
    {
      log_i("disconnecting...");
      client.disconnect();
    }
    log_i("waiting for disconnection...");
    while (client.isConnected())
      delay(100);
    log_i("Client disconnected.");
  }
  else
  {
    // We fail to connect and we have to be sure the PeerDevice is removed before delete it
    BLEDevice::removePeerDevice(client.m_appId, true);
    log_i("-------------------Not connected!!!--------------------");
  }

  log_i("<< ------------------batteryLevel----------------- ");
  return bleconnected;
}



ble_manager::ble_manager(uint32_t maxDevices)
{
  BLETrackedDevices.reserve(maxDevices);
  too_far_devices = 0;
  DEBUG_PRINTF("BLE reserved memory for %d devices\n", maxDevices);
}

ble_manager::~ble_manager()
{
  // TBD  desalocation vecteur ?
}

void  ble_manager::addKnownDevice(BLETrackedDevice ltrackedDevice){
   BLETrackedDevices.push_back(std::move(ltrackedDevice));
}

uint32_t ble_manager::NumDevicesTracked (){
  return BLETrackedDevices.size();
}
#endif

void ble_manager::init(String BLEname)
{
    BLEDevice::init(BLEname.c_str());
    
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(this), NUM_OF_ADVERTISEMENT_IN_SCAN > 1);
    pBLEScan->setActiveScan(ACTIVE_SCAN);
    pBLEScan->setInterval(50);
    pBLEScan->setWindow(50);

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_BT);
    Serial.print("Bluetooth MAC: ");
    for (int i = 0; i < 5; i++) {
      Serial.printf("%02X:", baseMac[i]);
    }
    Serial.printf("%02X\n", baseMac[5]);
  
}

void ble_manager::scan()
{

DEBUG_PRINTF("Number device discovered: %d ignored: %d\n", NumDevicesTracked() ,too_far_devices);

#if PROGRESSIVE_SCAN
      bool scanCompleted = false;

        static uint32_t elapsedScanTime = 0;
        static uint32_t lastScanTime = 0;

        bool continuePrevScan = elapsedScanTime > 0;
        if (!continuePrevScan) // new scan
        {
          // Reset the states of discovered devices
          too_far_devices=0;
          for (auto &trackedDevice : BLETrackedDevices)
          {
            trackedDevice.advertised = false;
            trackedDevice.rssiValue = MIN_BLE_rssiValue;
            trackedDevice.advertisementCounter = 0;
          }
        }

        lastScanTime = NTPTime::seconds();
        pBLEScan->start(1, continuePrevScan);
        pBLEScan->stop();
        elapsedScanTime += NTPTime::seconds() - lastScanTime;
        scanCompleted = elapsedScanTime > SettingsMngr.scanPeriod;
        if (scanCompleted)
        {
          elapsedScanTime = 0;
          pBLEScan->clearResults();
        }
#else
        // Reset the states of discovered devices
        too_far_devices=0;

        for (auto &trackedDevice : BLETrackedDevices)
        {
          trackedDevice.advertised = false;
          trackedDevice.rssiValue = MIN_BLE_rssiValue;
          trackedDevice.advertisementCounter = 0;
        }

        DEBUG_PRINTF("*** Memory Before scan: %u\n",xPortGetFreeHeapSize());
        pBLEScan->start(SettingsMngr.scanPeriod);
        // pBLEScan->stop();  // useless
         pBLEScan->clearResults();
        DEBUG_PRINTF("*** Memory After scan: %u\n",xPortGetFreeHeapSize());

#endif
}

void ble_manager::clear(){
        for (auto &trackedDevice : BLETrackedDevices)
        {
          trackedDevice.advertised = false;
          trackedDevice.rssiValue = MIN_BLE_rssiValue;
          trackedDevice.advertisementCounter = 0;
        }
        pBLEScan->clearResults();
}

void ble_manager::cleanList (u_int32_t maxAge){
      for (auto &trackedDevice : BLETrackedDevices)
      {
        if (trackedDevice.isDiscovered && (trackedDevice.lastDiscoveryTime + maxAge) < NTPTime::seconds())
        {
          trackedDevice.isDiscovered = false;
          FastDiscovery[trackedDevice.address] = false;
          LOG_TO_FILE_D("Devices %s is gone out of range", trackedDevice.address);
        }
      }

}

void ble_manager::getBatteryLevel()
{
   DEBUG_PRINTF("*** Memory before battery scan: %u\n",xPortGetFreeHeapSize());

  for (auto &trackedDevice : BLETrackedDevices)
  {
    if (!(SettingsMngr.InBatteryList(trackedDevice.address) || trackedDevice.forceBatteryRead))
      continue;

#if USE_MQTT
    if (!IsAccessPointModeOn())
    {
      publishAvailabilityToMQTT();
    }
#endif

    // We need to connect to the device to read the battery value
    // So that we check only the device really advertised by the scan
    unsigned long BatteryReadTimeout = trackedDevice.lastBattMeasureTime + BATTERY_READ_PERIOD;
    unsigned long BatteryRetryTimeout = trackedDevice.lastBattMeasureTime + BATTERY_RETRY_PERIOD;
    unsigned long now = NTPTime::getTimeStamp();
    char Caddress[ADDRESS_STRING_SIZE+5];
    CanonicalAddress(trackedDevice.address,Caddress );
    bool batterySet = trackedDevice.batteryLevel > 0;
    if (trackedDevice.advertised && trackedDevice.hasBatteryService && trackedDevice.rssiValue > -90 &&
        ((batterySet && (BatteryReadTimeout < now)) ||
         (!batterySet && (BatteryRetryTimeout < now)) ||
         trackedDevice.forceBatteryRead))
    {
      bool connectionEstablished = batteryLevel(trackedDevice.address, trackedDevice.addressType, trackedDevice.batteryLevel, trackedDevice.hasBatteryService);
      if (connectionEstablished || !trackedDevice.hasBatteryService)
      {
        log_i("Device %s has battery service: %s", trackedDevice.address, trackedDevice.hasBatteryService ? "YES" : "NO");
        DEBUG_PRINTF("Reading Battery level for %s (%s): Retries: %d %s\n", Caddress, trackedDevice.dshortName,
            trackedDevice.connectionRetry, trackedDevice.hasBatteryService ? "YES" : "NO");
 
        trackedDevice.connectionRetry = 0;
        trackedDevice.lastBattMeasureTime = now;
      }
      else
      {
        trackedDevice.connectionRetry++;
        DEBUG_PRINTF("Error: Connection to device %s (%s) failed\n", Caddress, trackedDevice.dshortName);
       if (trackedDevice.connectionRetry >= MAX_BLE_CONNECTION_RETRIES)
        {
          trackedDevice.connectionRetry = 0;
          trackedDevice.lastBattMeasureTime = now;
          // Report the error only one time if Log level info is set
          LOG_TO_FILE_E("Error: Connection to device %s failed", trackedDevice.address);
         }
        else
        {
          // Report the error every time if Log level debug or verbose is set
          LOG_TO_FILE_D("Error: Connection to device %s failed", trackedDevice.address);
        }
      }
    }
    else if (BatteryReadTimeout < now)
    {
      // Here we preserve the lastBattMeasure time to trigger a new read
      // when the device will be advertised again
      trackedDevice.batteryLevel = -1;
    }
    trackedDevice.forceBatteryRead = false;
    Watchdog::Feed();
  }
   DEBUG_PRINTF("*** Memory after battery scan: %u\n",xPortGetFreeHeapSize());

}