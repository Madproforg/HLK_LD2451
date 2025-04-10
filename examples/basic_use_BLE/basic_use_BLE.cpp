/**
 * Simple exmaple of using the HLK_LD2451 via BLE or serial
 * comment 
 */
//#define  SERIAL_CONNECTION
#define  BLE_CONNECTION
#include <Arduino.h>
#if defined(SERIAL_CONNECTION) 
    #include <HardwareSerial.h>

    // can define these before including HLK_LD2451.h if not using pins 15 & 16
    // if not defined the header sets them
    // can be overridden in the a begin() call
    //#define RADAR_TX 16
    //#define RADAR_RX 15
#else
    #if defined(USE_NIMBLE_LIBRARY)
    #include <NimBLEDevice.h>
    #else
    #include <BLEDevice.h>
    #endif
#endif

#include "HLK_LD2451.h"

#if defined(SERIAL_CONNECTION)

// which uart to use for the sensor
HardwareSerial uart2(2);

// sensor initial config
HLK_LD2451 ld2451_sensor(&uart2);
#else
// bt address of sensor
#if defined(USE_NIMBLE_LIBRARY)
static NimBLEAddress sensorAddress(std::string("57:01:f2:86:ac:c9"), 0);
#else
static BLEAddress sensorAddress("57:01:f2:86:ac:c9");
#endif
// sensor initial config
HLK_LD2451 ld2451_sensor;
#endif

void setup() {
    Serial.begin(115200);

    // wait for serial to init
    while (!Serial) vTaskDelay(10);

    // send sensor debugging output to Serial monitor
    ld2451_sensor.debugOutput(&Serial);

    // begin sensor reading with defaults pins 15 for RX  16 for TX
#if defined(SERIAL_CONNECTION)
    // begin sensor reading with defaults pins 15 for RX  16 for TX
    ld2451_sensor.begin();

    // begin sensor reading with custom serial paramaters
    //ld2451_sensor.begin(115200, SERIAL_8N1, 15, 16);
#else
    // begin ble connection to sensor
    ld2451_sensor.begin_BLE(&sensorAddress);
#endif

}

void loop() {
    // wait 1 second for a detection event
    if (ld2451_sensor.waitForDetection(1000)) {
        // detected vehicles are in a vector - no manual memory management
        std::vector<LD2451::vehicleTarget_t> detected = ld2451_sensor.getTargets();
        
        // output all detected items to serial monitor
        Serial.println("--------------------");
        Serial.printf("%d vehicles detected\r\n", detected.capacity());
        uint8_t vehicleNumber = 1;
        for (LD2451::vehicleTarget_t singleTarget : detected) {
            Serial.printf("vehicle #%d\r\n", vehicleNumber);
            Serial.printf("Distance: %dm\r\n", singleTarget.distance);
            Serial.printf("@angle: %d°\r\n", singleTarget.angle);
            Serial.printf("Heading %s\r\n", singleTarget.direction==0 ? "Away" : "Towards");
            Serial.printf("With a speed of %d km/h\r\n", singleTarget.speed);
            Serial.printf("SNR: %d\r\n", singleTarget.snr);
            Serial.println("-----");
            vehicleNumber++;
        }
    }
}