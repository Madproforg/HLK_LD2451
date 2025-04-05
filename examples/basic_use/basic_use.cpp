#include <Arduino.h>
#include <HardwareSerial.h>

// define these before including HLK_LD2451.h if not using pins 15 & 16
// if not defined the header sets these
// can be overridden in the a begin() call
//#define RADAR_TX 16
//#define RADAR_RX 15

#include <HLK_LD2451.h>

// which uart to use for the sensor
HardwareSerial uart2(2);

// sensor initial config
HLK_LD2451 ld2451_sensor(uart2);

void setup() {
    Serial.begin(115200);

    // wait for serial to init
    while (!Serial) vTaskDelay(10);

    // send sensor debugging output to Serial monitor
    ld2451_sensor.debugOutput(&Serial);

    // begin sensor reading with defaults
    ld2451_sensor.begin();

    // begin sensor reading with custome serial paramaters
    //ld2451_sensor.begin(115200, SERIAL_8N1, 15, 16);
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
        }
    }
}