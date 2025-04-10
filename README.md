ESP32 (Arduino code base) only library for the Highlink LD2451 sensor

Inspired by the discussion from the Arduino forums - https://forum.arduino.cc/t/using-hlk-ld2451-radar-for-speed-measurment/1328105
and from the LD2410 library - https://github.com/ncmreynolds/ld2410

Can connect to the sensor via uart(Serial) or BLE

BLE can be switched between ESP default(bluedroid) or NimBLE-Arduino (https://github.com/h2zero/NimBLE-Arduino)

The default ble has an issue with S3 - on connect it overflows a stack and core dumps
use the pioarduino port of the new espressif sdk for platformio.
I haven't tried on other ESP32 boards.

[env:....]
platform = https://github.com/pioarduino/platform-espressif32.git#develop

Note: at the current time if the BLE connection drops the esp32 has to be restarted/power cycled

Add a 
build_flag = -DUSE_NIMBLE_LIBRARY 
and 
lib_deps = 
    h2zero/NimBLE-Arduino@^2.2.3
    
to platformio.ini

the flag will toggle the code to NimBLE-Arduino

NimBLE-Arduino uses up less of the ram/flash storage
memory usage for basic_use_BLE examples compiled for an ESP32 S3
Default BLE
RAM:   [=         ]  14.8% (used 48468 bytes from 327680 bytes)
Flash: [=         ]  14.4% (used 940634 bytes from 6553600 bytes)

NimBLE-Arduino
RAM:   [=         ]   9.7% (used 31704 bytes from 327680 bytes)
Flash: [=         ]   8.7% (used 572974 bytes from 6553600 bytes)