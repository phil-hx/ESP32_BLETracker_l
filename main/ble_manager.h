#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <BLEDevice.h>
#include "myRWMutex.h"

#include "utility.h"


struct BLETrackedDevice
{
  char address[ADDRESS_STRING_SIZE];
  bool isDiscovered; //Until it's TRUE the device is considered Online, if it's not discovered for a period it become FALSE
  long lastDiscoveryTime;
  long lastBattMeasureTime;
  int8_t batteryLevel;
  bool advertised;         //TRUE if the device is just advertised in the current scan
  bool hasBatteryService;  //Used to avoid connections with BLE without battery service
  uint8_t connectionRetry; //Number of retries if the connection with the device fails
  int8_t rssiValue;
  esp_ble_addr_type_t addressType;
  uint8_t advertisementCounter;
  bool forceBatteryRead;
  char dshortName [MAX_Dname+1];

  BLETrackedDevice()
  {
    address[0] = '\0';
    isDiscovered = 0;
    lastDiscoveryTime = 0;
    lastBattMeasureTime = 0;
    batteryLevel = -1;
    advertised = false;
    hasBatteryService = true;
    connectionRetry = 0;
    rssiValue = -100;
    addressType = BLE_ADDR_TYPE_PUBLIC;
    advertisementCounter = 0;
    forceBatteryRead = true;
    dshortName[0] = '\0';
  }
};


class ble_manager
{
public:
    ble_manager(uint32_t maxDevices);
    ~ble_manager();

    void init( String BLEname);

    void addKnownDevice(BLETrackedDevice ltrackedDevice);

    uint32_t NumDevicesTracked ();

    void scan();

    void clear ();

    void cleanList (u_int32_t maxAge);

    void  getBatteryLevel();

private:

    BLEScan *pBLEScan;
    int too_far_devices ;

    std::vector<BLETrackedDevice> BLETrackedDevices;

    MyRWMutex trackedDevicesMutex;

    std::map<std::string, bool> FastDiscovery;

    void ForceBatteryRead(const char *normalizedAddr);

    friend class MyAdvertisedDeviceCallbacks;
    friend class OTAWebServer;
 
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
public:
  MyAdvertisedDeviceCallbacks(ble_manager * pBLE);
  
  void onResult(BLEAdvertisedDevice advertisedDevice) override;

private:
   ble_manager  *pble_manager;
};

#endif