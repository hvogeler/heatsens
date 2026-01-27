# Deep Sleep between measurements

# Goal

I want to make this heat sensor more power efficient. Potentially running it on battery if we can get to a decent battery life span.

# What the heat sensor does

This heat sensor does 3 things:

1. At regular time intervals the sensor reports its measurements to mqtt "heatsens/cur_temp/$device-id" topic
2. It compares current temperature with a target temperature and if target temperature is higher it sets the "is_heating_requested" variable of the json payload to true.
3. It subscribes to the "heatctrl/infofor/$device-id" topic to see if the heat actuator it is supposed to trigger is turned on or not. If it is turned on it displays the state on its lcd display
4. It subscribes to the "heatsens/tgt_temp/$device-id" topic to receive target temperature updates (stored in NVS)

# Optimize Power Consumption

Between the measurements the heat sensor could deep sleep. When it wakes up it could:

1. Read the cur_temp from the bmp280 and send the message to heatsens/cur_temp/$device-id
2. Read from the heatctrl/infofor/$device-id topic to determine the actuator state
3. Read from the heatsens/tgt_temp/$device-id topic to get any target temperature updates
4. Go back to sleep

Problem is that wifi radio gets powered down during deep sleep. So we need to implement a clean way to

## 1. prepare deep sleep

1. shut down mqtt (which uses wifi)
2. shut down wifi

## 2. wake up cleanly

1. start wifi and connect to the wlan
2. start mqtt

# Problem

So far I was not able to shutdown mqtt and wifi in a way that it restarts cleanly on wakeup.

---

# Implementation Decisions

## Scope

The current focus is **only** on getting deep sleep to work reliably with clean WiFi/MQTT shutdown and restart. All other features are deferred to future iterations.

## Sleep Duration

- Use a fixed constant value (no configuration via menuconfig for now)
- For testing: **10 seconds** (short enough to iterate quickly)
- Production value (later): ~10 minutes

## Display

- Currently using LilyGo T-Display-S3 (LCD)
- E-paper display integration is deferred to a future step (more power efficient)
- Display handling during sleep is not a priority for this phase

## Motion Sensor (MPU6050)

- Motion-based wake-up is deferred to a future step
- Current implementation: **timer-based wake-up only**

## State Persistence

- All necessary state is already persisted in NVS (namespaces: "config" and "meta")
- Target temperature (tgt_temp) is stored in NVS when received via MQTT
- No additional RTC memory storage needed

## Mode Switching

- No normal/sleep mode toggle for now
- Future: configurable via MQTT topic (similar to heatsens/tgt_temp/$device-id)

## Subscription Handling

- After wake-up: connect, publish, subscribe briefly, then sleep
- Do **not** wait for subscription responses
- Actuator state will be read on the next wake cycle
