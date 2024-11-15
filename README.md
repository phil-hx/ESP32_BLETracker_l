# Table of Contents

- [ESP32 BLETracker](#esp32-bletracker)
  - [WEB Server](#web-server)
  - [FHEM Support](#fhem-support)
- [Arduino IDE Notes (outdated)](#arduino-ide-notes-outdated)
- [Have you found a crash? Help me to fix it.](#have-you-found-a-crash-help-me-to-fix-it)
- [Tested BLE Devices](#tested-ble-devices)
- [Home Assistant integration](#home-assistant-integration)
- [Licence](#licence)
- [Support my work](#support-my-work)
  <br /><br />

# ESP32 BLETracker

A simple example describing how to track a Bluetooth Low Energy device with an ESP32, the MQTT protocol and Home Assistant. Please note that the targeted device can't have a changing BLE address (normally called random instead of public address).

Use [PlatformIO](https://platformio.org/) to build and deploy this application, remember to install [git](https://git-scm.com/downloads) in order to allow PlatformIO to download automatically all the required dependencies.<br />
You have to modify the **user_config.h** file inserting the correct information to connect to the WiFi and to the MQTT broker.<br />
From version 3.7, the ESP32 starts in access point mode with the IP address 192.168.2.1 if WiFi connection information has not been provided. WiFi network credentials can be entered directly through the web interface on the configuration page.<br />
The GATEWAY_NAME is used as Client ID to connect to the broker so be sure it's unique.<br />

The battery level can be read from the devices providing the Battery Service (0x180F) and the Battery Level characteristic (0x2A19), check the availability using a nRF Sniffer i.e. [nRF Connect](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp) <br />
This feature was successfully tested with a Nut Mini, using other devices you could have connection problems.<br />
If many devices are discovered the battery level check can be very slow causing frequent Wi-Fi disconnection so that I have introduced a whitelist containing the Mac Address of the devices to check. The whitelist is in the form:<br />
BLE_BATTERY_WHITELIST       "XXXXXXXXX","YYYYYYYY"<br />
Mac Addresses have to be uppercase without ":" or "-" i.e "BA683F7EC159"

The application can generate the following topics:<br />
A topic for the BLETracker state:<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS&gt;/LWT payload: &lt;online|offline&gt;<br /><br />
A single topic with the payload in JSON format containing all the items returned by the device (this is the default):<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS payload: { "state":&lt;"on"|"off"&gt;,"rssi":&lt;dBvalue&gt;,"battery":&lt;batterylevel&gt;}<br /><br />
A topic for each item returned by the advertised device:<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS&gt;/state payload: &lt;"on"|"off"&gt;<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS&gt;/rssi payload: &lt;dBvalue&gt;<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS&gt;/battery payload: &lt;batterylevel&gt;<br /><br />
A topic with helpfull system information:<br />
&lt;LOCATION&gt;/&lt;GATEWAY_NAME&gt;/&lt;BLE_ADDRESS&gt;/sysinfo, payload: { "uptime":&lt;timesinceboot&gt;,"version":&lt;versionnumber&gt;,"SSID":&lt;WiFiSSID&gt;,"IP":&lt;ipnumber&gt;}

### WEB Server

A WEB server is integrated into the BLETracker, it can be accessed using a web browser and the ip or the network name of the tracker.<br />
The WEB server can be used to see some system information and to update the firmware using an **OTA Update**, simply choosing the .bin file to upload.<br />
Default credential to access the WEB Server are:<br />
user: admin <br />
password: admin <br />
<br />
The new WEB server interface allow to customize the list of devices to be tracked, the scan period, and the MQTT broker parameters.<br />
The new interface allows also to monitor some logs. The number of logs is limited and when the maximum capacity is reached the oldest are removed.<br />
<br />
You can also enable manual scanning. You can start and stop scanning using the following API:

```
GET http://<user>:<password>@<device_ip>/scan?on=1
GET http://<user>:<password>@<device_ip>/scan?off=1
```

### FHEM Support

If properly configured the BLETracker can be integrate in your FHEM environment. Instead publishing MQTT tokens the BLETracker can be configured to act as a lepresenced daemon.
A server will run on the port 5333 listening for incoming connection generated by the collectord. The FHEM support and the MQTT support are mutually exclusive because of memory issues. The FHEM support can be easily enabled using Platform.io simply choosing the **esp32dev-ble-fhem-release** build variant.<br />
The values of *presence_timeout* and *absence_timeout* stored in the collectord.conf cannot be less than or equal to the BLE_SCANNING_PERIOD (default is 10 seconds).

# Arduino IDE Notes (outdated)

You can build this sketch using Arduino IDE (currently it's using arduino-esp32 v1.0.4), but be sure to install the required dependencies:<br />

* You have to install the esp32 Board in Arduino IDE. You can find a tutorial following this [link](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)
* You have to install the library PubSubClient v2.8 (not needed if you want enable FHEM support)
* To enable the FHEM support you have to:
  * install the Regexp library (https://www.arduino.cc/reference/en/libraries/regexp/)
  * set USE_MQTT to false and USE_FHEM_LEPRESENCE_SERVER to true in the config.h
* Because a bug in the BLE library provided by arduino-esp32 v1.0.4 you have to replace it with this one https://github.com/shogunxam/ESP32-BLE-Arduino.<br />
  **Since the release v2.1 this one must be used https://github.com/espressif/arduino-esp32/tree/6b0114366baf986c155e8173ab7c22bc0c5fcedc/libraries/BLE.**<br />
  The libray to replace should be located in the folder ~/.arduino15/packages/esp32/hardware/esp32/x.x.x/libraries/BLE for Unix users and in C:\Users\YourUserName\AppData\Local\Arduino15\packages\esp32\hardware\esp32\x.x.x\libraries\BLE for Windows users.<br />

Build using the *Minimal SPIFFS* partition schema.

# Have you found a crash? Help me to fix it.

In order to understand the cause of the crash I need the backtrace returned by the firmware.

- First you have to build the firmware in debug mode, if you are using PlatformIo you can switch to the proper Project environment clicking on the env:&lt;enviornemnt_name&gt; button placed on the PlatformIo status bar.
- Flash the built firmware and open the console monitor.
- Execute the procedure causing the crash.
- When the crash occurs on the console you will see something like:

```
Backtrace: 0x4008505f:0x3ffbe350 0x401b9df5:0x3ffbe370 0x40019fb5:0x3ffbe390 0x40089692:0x3ffbe3c0 0x40088015:0x3ffbe400 0x40087d31:0x3ffbe420 0x4008a422:0x3ffbe440 0x4008b553:0x3ffbe460 0x400816e9:0x3ffbe480 0x401ba224:0x3ffe52a0 0x401ba7ed:0x3ffe52c0 0x4009008a:0x3ffe52f0
```

- Copy and paste this line in a file (i.e. backtrace.txt)
- Open a terminal, if you are using PlatformIo simply select bash from the Terminal window
- Run the `decoder.py` script you find in the project (it needs you have installed Python 3, if you are using PlatformIo you are fine). Refer to [EspArduinoExceptionDecoder](https://github.com/me21/EspArduinoExceptionDecoder) for more information on the tool.

```
usage: decoder.py [-h] [-p {ESP8266,ESP32}] [-t TOOL] -e ELF [-f] [-s] file

decode ESP Stacktrace.

positional arguments:
  file                  The file to read the exception data from ('-' for STDIN)

optional arguments:
  -h, --help            show this help message and exit
  -p {ESP8266,ESP32}, --platform {ESP8266,ESP32}
                        The platform to decode from
  -t TOOL, --tool TOOL  Path to the xtensa toolchain
  -e ELF, --elf ELF     path to elf file
  -f, --full            Print full stack dump
  -s, --stack_only      Decode only a stacktrace
```

so you have to write something like:

```
./decoder.py -p ESP32 -t ~/.platformio/packages/toolchain-xtensa32 -e .pio/build/esp32dev-ble-debug/firmware.elf ./backtrace.txt 
```

- Open an issue giving as much as possible details you can and attach the output of the decoder.
  <br /><br />

# Tested BLE Devices

| BLE Device            | Discovery | Battery |
| --------------------- | --------- | ------- |
| Nut mini              | ✔️      | ✔️    |
| Nut2                  | ✔️      | ❗️    |
| Remote Shutter        | ✔️      | ✔️    |
| Xiomi Amazfit Bip     | ✔️      | ❌      |
| REDMOND RFT-08S       | ✔️      | ❌      |
| Xiomi Mi Smart Band 4 | ✔️      | ❌      |
| Fitness Band GT101    | ✔️      | ❌      |

# Home Assistant integration

This is a simple example of a package to manage a Nut Tracker device.<br />
A more complex example combining more BLETrackers can be found inside the Doc folder.<br />
![Alt text](/image.png?raw=true "Screenshot")

```
###################################################
#Sensors
sensor:
- platform: mqtt
  state_topic: 'home/BLETracker/XXXXXXXXXXXX'
  name: 'bt_nut_upstairs_state' 
  value_template: "{{ value_json.state }}"
  expire_after: 120 
  force_update: true

- platform: mqtt
  state_topic: 'home/BLETracker/XXXXXXXXXXXX'
  name: 'bt_nut_upstairs_battery' 
  value_template: "{{ value_json.battery }}"
  force_update: true  

###############################################
#Battery Sensor to resolve unknown/unavailable device state
- platform: template
  sensors:
    bt_nut_battery:
      unit_of_measurement: '%'
      value_template: >
          {% if (is_state('sensor.bt_nut_upstairs_battery', 'unknown') or is_state('sensor.bt_nut_upstairs_battery', 'unavailable') or is_state('sensor.bt_nut_upstairs_battery', '-1')) %}
            unknown
          {% else %}
            {{ states('sensor.bt_nut_upstairs_battery')|int }}
          {% endif %}
      icon_template: >
        {% set battery_level = states('sensor.bt_nut_battery.state')|default(0)|int %}
        {% set battery_round = (battery_level / 10) |int * 10 %}
        {% if battery_round >= 100 %}
          mdi:battery
        {% elif battery_round > 0 %}
          mdi:battery-{{ battery_round }}
        {% else %}
          mdi:battery-alert
        {% endif %}


###############################################
#Binary Sensors to resolve unknown/unavailable device state

binary_sensor:
- platform: template
  sensors:
    ble_tracker_nut_mario:
      friendly_name: 'ble_tracker_nut'
      value_template: >
        {% if (is_state('sensor.bt_nut_upstairs_state', 'unknown') or is_state('sensor.bt_nut_upstairs_state', 'unavailable') or is_state('sensor.bt_nut_upstairs_state', 'off')) %}
          false
        {% else %}
          true
        {% endif %}
      device_class: presence
      delay_off: 
        minutes: 1
    
#######################
#Check BLETracker state
- platform: mqtt
  name: BLETracker_state
  state_topic: 'home/BLETracker/LWT'
  payload_on: "online"
  payload_off: "offline"
  device_class: "connectivity"

########################
#Automations
automation:
- id: notify_BLETracker_state
  alias: Notify BLETracker offline
  initial_state: true
  trigger:
  - platform: state
    entity_id: binary_sensor.BLETracker_state
    to: 'off'
    for:
      minutes: 5 
  action:
  - service: notify.telegram
    data:
      title: BLETracker
      message: "BLETracker offline"

- id: notify_nut_battery
  alias: Notify Battery low
  initial_state: true
  trigger:
  - platform: template
    value_template: >
      {{ ((states('sensor.bt_nut_battery') | int ) <= 20 ) and ( is_state('sensor.bt_nut_battery', 'unknown') or  is_state('sensor.bt_nut_battery', 'unavailable')) and ( states('sensor.bt_nut_battery') | int >= 0) }}
  action:
  - service: notify.telegram
    data_template:
      title: BLETracker
      message: "Nut battery at {{states('sensor.bt_nut_battery')}}%"

```

Alternatively you can use the single topic returning the state in the following way:

```
- platform: mqtt
  name: "bt_nut_upstairs_stat"
  state_topic: "home/BLETracker/XXXXXXXXXXXX/state"
  availability_topic: "home/BLETracker/LWT"
  expire_after: 300
  value_template: >-
     {% if value == 'on' %}
        home
     {% elif value == 'off' %}
        away
     {% else %}
        away
     {% endif %}
```

# Licence

Permission is hereby granted, free of charge, to any person obtaining a copy <br />
of this software and associated documentation files (the "Software"), to deal <br />
in the Software without restriction, including without limitation the rights <br />
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell <br />
copies of the Software, and to permit persons to whom the Software is <br />
furnished to do so, subject to the following conditions:<br />
<br />
The above copyright notice and this permission notice shall be included in all <br />
copies or substantial portions of the Software.<br />
<br />
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR <br />
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,<br />
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE <br />
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER <br />
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,<br />
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE <br />
SOFTWARE.<br />

*If you like the content of this repo, please add a star! Thank you!*

# Support my work

If you like my work, please consider buying me a coffee or making a free donation via PayPal. Thank you for your support! 😁<br /><br />
[Buy me a coffee][buymeacoffee] [PayPal][paypal]

[buymeacoffee-shield]: https://raw.githubusercontent.com/shogunxam/donate-assets/master/coffee.png
[buymeacoffee]: https://www.buymeacoffee.com/shogunxam
[paypal-shield]: https://raw.githubusercontent.com/shogunxam/donate-assets/master/paypal.png
[paypal]: https://paypal.me/shogunxam
