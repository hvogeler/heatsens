# NVS Use

Essential network parameters like

- wifi-ssid
- wifi-password
- mqtt_broker
- mqtt_user
- mqtt_password
  are stored in NVS.

Initially they can be flashed to NVS.

# Flashing NVS configuration

Configure configuation parameters in nvs_creds.csv. Then run

```
flash_nvs.sh
```

This will generate a binary file from the input csv and flash it to NVS memory.
Check usb mount point in flash_nvs.sh because it can change.

# Web Provisioning

If NVS is not configured/flashed the controller will enter web provision on startup.
This will start a wifi AP on 192.168.4.1 and provide a html page that allows the user to configure the wifi and mqtt credentials.

# Clear NVS

Run `idf.py partition-table` and check the NVS partition's boundaries:
For example:

```
nvs,data,nvs,0x9000,24K,
```

Then erase it (Note that 0x6000 == 24K)

```
esptool.py erase_region 0x9000 0x6000
```
