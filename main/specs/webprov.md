# Specs for web based provisioning

I want to create a specification so you can generate web based provisioning for this esp32 based project.

# First requirements

## Component

make this provisioning manager a separate component so I can easily re-use it in other projects.

## WiFi AP

Create an AP using the CONFIG*HEATSENS_DEVICE_ID from sdkconfig as a postfix to the ssid. Make the ssid `${PROJECT_NAME}*${CONFIG_HEATSENS_DEVICE_ID}`. If the CONFIG_HEATSENS_DEVICE_ID=1003 and the project name is "heatsens" the ssid should be "heatsens_cfg_1003". If getting the project name from the CMakeLists file is too much coding (like more then 5 lines of code) let us add the project name to sdkconfig.

Access to the AP should be as simple as possible. We do not care about security for provisioning.

## Http Server

Provisioning Manager should start an Http server that servers 2 things:

1. a service HTTP_POST endpoint /config that allows to simply configure the device using a POST request
2. a html web form that asks for the configurartion parameters

We do not want ssl. Security is not an issue for our provisioning.

### Http Endpoints

#### POST /config

Create an http server that provides a POST endpoint /config that takes this payload:

```
{
  "wifi_ssid": string,
  "wifi_password": string,
  "mqtt_broker": string,
  "mqtt_user": string,
  "mqtt_password": string
}
```

#### POST /cancel

This endpoint ends the provisioning process. See explanation how to end the provisioning process below.

### Html Form

present a html form on the root uri of port 80 that displays a form that asks for the parameters. In this case it should be the fields we use for the POST payload of the api.

### Parameter Persistance

The provisioned parameters must be stored in nvs namespace 'config' using the same field names as the payload above.

### Progressicve Webapp

The web form must be designed mobile first. I assume that most people will use a phone to provision the device.

# Provisioning Workflow

## Initialization

If the device is powered on the first time the provisioning process should automatically be started.

## Update

if the user presses the BOOT button longer the 3 seconds WHILE the device is running, the provisioning process should be entered with blanked fields in the UI. As long as no new parameters are posted the old nvs stays unchanged. When new parameters are posted they replace the nvs parameters. The HTML page must provide a 'Cancel' button to bail out of provisioning without making any changes and reboot the device.

## End

Provisioning mode is exited whenever either parameter values are posted, or the user cancels. There is no time out. The device waits for ever until one of the above conditions happens.
