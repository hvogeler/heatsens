# heatsens

`. /Users/hvo/esp/esp-idf/export.sh`

# Overview

This firmware implements a heat sensor.

# Configure Credentials

This sensor needs credentials

- to connect to wifi
- to connect to the mqtt broker

The firmware expectes these credentials in NVS ram. This means at some point we need to flash them to NVS before the firmware reads them.

## Configure Values

Copy the nvs_template.csv to nvs_creds.csv.

```
key,type,encoding,value
config,namespace,,
wifi_ssid,data,string,YOUR_WIFI_SSID
wifi_password,data,string,YOUR_WIFI_PASSWORD
mqtt_broker,data,string,mqtt://YOUR_BROKER:1883
mqtt_user,data,string,YOUR_USER
mqtt_password,data,string,YOUR_PASSWORD
```

Replace the placeholdes with the actual credentials.

Now run `./flash_nvs.sh`. You might need to adjust the serial port.
On my MAC it is `/dev/cu.usbmodem1101`.

This flashes the credentials to the devices NVS where they will stay until the flash is cleared.
Firmware updates will not clear the credentials.

# Topics
```
heatctrl/infofor/Flur
{
 "is_heating": 0
}
```
```
heatsens/tgt_temp/Flur
22.0
```