# Room temperature sensor (heatsens)

# Overview

This firmware implements a heat sensor. It is part of a complete floor heating system for my apartement.
This firmware is meant to run on the Lilygo t-display-s3. It uses a BME280 sensor to measure current room temperature.
It will check current temperature against a set target temperature and trigger heating of its room.

All communication is done over mqtt5. Temperatures are always in degrees celsius.

# Process Details

## Subscribed Topics

### Topic heatctrl/infofor/Flur

qos 1, retained true

The central controller publishes its heating state for the room on this topic. 'Flur' is the name of the room.

Payload:

```
heatctrl/infofor/Flur
{
 "is_heating": 0
}
```

0 - is not heating
1 - it is heating

The sensor displays this information to the user.

### Topic heatsens/tgt_temp/Flur

qos 1, retained true

This topic is used to configure the target temperature for this room. In this example the room is 'Flur'.
The target temperature could be set by a home automation platform's UI.

Payload:

```
21.0
```

This sets the target temperature to 21.0 degrees celsius.

## Published Topics

### heatsens/cur_temp/Flur

qos 1, retained false

The sensor publishes

- its current temperature
- its target tempreature
- a heating request flag
  on this topic. 'Flur' is the name of the room.

```
{
	"cur_temp":	23.1251163482666,
	"tgt_temp":	24,
	"is_heating":	true,
	"ts":	"2025-11-27T13:09:23+0100"
}
```

The central controller is expected to obey the is_heating flag and turn on the heating valve for this room.

# Calculating heating flag
If the target temperature is greater than the current temperature + histeresis, the __is_heating__ flag is set to true.
If the current temperature is greater than the target temperature + histeresis, the __is_heating__ flag is set to false.
The histeresis value can be configured. Default is 0.5 degrees Celsius. 

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
