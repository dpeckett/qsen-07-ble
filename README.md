# Qsen-07 BLE

BLE [Environmental Sensing Service](https://www.bluetooth.com/specifications/specs/environmental-sensing-service-1-0/) firmware for the [Qsen-07](https://aliexpress.com/item/1005007922381128.html) environmental sensor module.

## Build and flash

Connect via USB type C cable. Then run:

```shell
cargo esp32c6
```

## Issues

* Temperature readings are off by approximately +3 degrees Celsius. This is likely 
due to self-heating of the sensor inside the enclosure.
* Light sensor readings have to compensate for the enclosure's transparent cover 
(blocks 80% of incoming light).