# Logging

This sensor should be part of my homelab o11y architecture.
This means it needs

- to log to a mqtt topic
- comply to the log topics nameing conventions
- use the logging structure

# Logging Architecture

All IoT sensors and IoT servers log to mqtt topics.
A o11y log provider needs to subscribe to all log topics and forward the messages to the o11y stack's logging which is Loki.
I think I will use fluent-bit as the log provider.

Fluent-bit has a source plugin that implements a mqtt broker itself.

My production broker forwards all log topics to the fluent-bit mqtt broker.
Fluent-bit then forwards the log messages to Loki adding topic information as labels to the log message.

## Log Topics' structure

```
log/<environment>/<application>/<device-type>/<deviceid>/<severity>
```

For example the heat sensor for the kitchen:

```
log/prod/floorheater/heatsens/kitchen/error
```

## Log Payload

```
{
   "ts": "2025-11-26T10:30:45.123Z",
   "device_type": "heatsens",
   "device_id": "kitchen",
   "severity": "error",
   "message": "No temperature reading from sensor",
}
```

## Enumerations

### Severity

- Error
- Warning
- Info
- Debug
- Trace

### Environment

- dev - Development
- test - Test
- prod - Production
