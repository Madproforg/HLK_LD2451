#if !defined(ESP32)
#error HLK_LD2451 is written for an ESP32 variant only - uses features not available on the standard arduino  cpus
#endif
/**
 * starting point for this code
 *  posts on the arduino forum https://forum.arduino.cc/t/using-hlk-ld2451-radar-for-speed-measurment/1328105
 * and the ld2410 library https://github.com/ncmreynolds/ld2410
 * 
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <memory>
#include <vector>
#if !defined(USE_NIMBLE_LIBRARY)
#include <BLEDevice.h>
#else
#include <NimBLEDevice.h>
#endif


#include "HLK_LD2451.h"

void hexout(byte *buffer, int len) {
    for (auto x=0; x<len; x++) {
        Serial.printf("%02X", *(buffer+x));
    }
}

namespace LD2451 {
    /**
     * returns a String for the cmd
     */
    String cmdName(LD2451::cmdValue cmd) {
        switch (cmd) {
            case CMD_enableConfig: return String("Enable Config");
            case CMD_endConfig: return String("End Config");
            case CMD_targetDetectionConfig: return String("Target Detection Config");
            case CMD_readFirmwareVersion: return String("Read Firmware Version");
            case CMD_readSensitivityConfig: return String("Read Sensitivity Config");
            case CMD_readTargetParameter: return String("Read Target Parameter ");
            case CMD_restartModule: return String("Restart Module");
            case CMD_restoreFactory: return String("Factory Reset");
            case CMD_sensitivityConfig: return String("Sensitivity Config");
            case CMD_setBaudrate: return String("Set Baud Rate");
        }
        return String("Unknown CMD: 0x%2x", cmd);
    }
}// end namespace

namespace HLK_LD2451BLE {
    // service which the sensor advertises
    static BLEUUID ld2451AdvertisedServiceUUID(BLEUUID((uint16_t)0xaf30));

    // general service to connect
    static BLEUUID ld2451UsageServiceUUID(BLEUUID((uint16_t)0xfff0));

    // BLE Characteristic for obtaining data from sensor
    static BLEUUID ld2451DataUUID(BLEUUID((uint16_t)0xfff1));

    // BLE Characteristic to write cmds to
    static BLEUUID ld2451CmdUUID(BLEUUID((uint16_t)0xfff2));

    // hacks to work around static onNotify callback
    HLK_LD2451 *owner = nullptr; 
    Stream* _debugUart = nullptr;
}// end namespace HLK_LD2451BLE

// millis to wait for ACK
#define HLK_LD2451_CMDACKWAIT 200

/**
 * Constructor for using uart interface 
 * */
HLK_LD2451::HLK_LD2451(HardwareSerial *Uart) : _radarSerial(Uart) {
    radarLock = xSemaphoreCreateMutex();
    ackWaiter = xSemaphoreCreateBinary();
    detectionWaiter = xSemaphoreCreateBinary();
    _debugUart = nullptr;
    firmware_data.type == 0;
}

/**
 * Constructor for using BLE interface
 */
HLK_LD2451::HLK_LD2451() : _radarSerial() {
    radarLock = xSemaphoreCreateMutex();
    ackWaiter = xSemaphoreCreateBinary();
    detectionWaiter = xSemaphoreCreateBinary();
    _debugUart = NULL;
    firmware_data.type == 0;
    usingBLE = true;
    HLK_LD2451BLE::owner = this;
}

/**
 * begin BLE
 * @param *pAddress - bt address of sensor to connect to
 * 
 * @return true - success  |  false - failed
 */
bool HLK_LD2451::begin_BLE(BLEAddress *pAddress) {
    usingBLE = true;
    BLEDevice::init("HLK_LD2451_READER");
    bleServerAddress = pAddress;
    if (_debugUart) _debugUart->printf("Connecting to BLE Sensor %s\r\n", bleServerAddress->toString().c_str());
    pClient = BLEDevice::createClient();
    if (_debugUart) _debugUart->println(" BLE client created");
    if (pClient->connect(*bleServerAddress)) {
        if (_debugUart) _debugUart->println(" BLE Connected to sensor");
    } else {
        if (_debugUart) _debugUart->println(" BLE failed to connect to sensor");
        return false;
    }
    

    if (_debugUart) _debugUart->println(" BLE Obtaining Service reference");
    sensorRemoteService = pClient->getService(HLK_LD2451BLE::ld2451UsageServiceUUID);
    if (sensorRemoteService == nullptr) {
        if (_debugUart) _debugUart->print(" Failed to obtain service reference UUID: ");
        if (_debugUart) _debugUart->println(HLK_LD2451BLE::ld2451UsageServiceUUID.toString().c_str());
        return false;
    }

    if (_debugUart) _debugUart->println(" BLE obtaining data characteristic");
    bleData = sensorRemoteService->getCharacteristic(HLK_LD2451BLE::ld2451DataUUID);
    if (bleData == nullptr) {
        if (_debugUart) _debugUart->println("Failed to obtain data characteristic");
        return false;
    }

    if (_debugUart) _debugUart->println(" BLE obtaining cmd characteristic");
    bleCmd = sensorRemoteService->getCharacteristic(HLK_LD2451BLE::ld2451CmdUUID);
    if (bleCmd == nullptr) {
        if (_debugUart) _debugUart->println("Failed to obtain cmd characteristic");
        return false;
    }

    if (_debugUart) _debugUart->println("Enabling ble notifications for data");
#if !defined(USE_NIMBLE_LIBRARY)    
    bleData->registerForNotify(newData);
#else
    bleData->subscribe(true, newData, true);
#endif

        // one shot task to read current paramaters from the sensor
    // without a delay between cmds something gets lost
    xTaskCreate(initParamsTask, "InitParams", 3000, this, 1, &initParamsTaskHandle);

    return true;
}

/**
 * called by the on_notify call back for initial data processing
 */
void HLK_LD2451::processBLEData(uint8_t* pData, size_t length) {
    if (length < 8) return;
    uint16_t datalength = pData[5] << 8 | pData[4];
    if (pData[0] == 0xF4 && pData[1] == 0xF3 && pData[2] == 0xF2 && pData[3] == 0xF1) {
        if (datalength) processTargets(datalength, pData+6);
    }
    if (pData[0] == 0xFD && pData[1] == 0xFC && pData[2] == 0xFB && pData[3] == 0xFA) {
        if (datalength) processOther(datalength, pData+6);
    }
}

/**
 * on_notify callback - called when sensor sends data
 */
void HLK_LD2451::newData(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
    if (HLK_LD2451BLE::_debugUart) {
        HLK_LD2451BLE::_debugUart->print("Data Received: ");
        for(auto i=0; i<length; i++) {
            HLK_LD2451BLE::_debugUart->printf("%02x", pData[i]);
            HLK_LD2451BLE::_debugUart->print(" ");
        }
        HLK_LD2451BLE::_debugUart->println();
    }
    HLK_LD2451BLE::owner->processBLEData(pData, length);
        

}

/**
 * Enabled serial debugging output
 * @param uart - point to initialised uart interface
 */
void HLK_LD2451::debugOutput(Stream *uart) {
    _debugUart = uart;
    HLK_LD2451BLE::_debugUart = uart;
}

/**
 * begin serial connection to sensor
 * @param baud - baud rate to use default 115200
 * @param config - serial config default SERIAL_8N1
 * @param rxPin - pin connected to sensors RX 
 * @param txPing - pin connected to sensors TX
 */
void HLK_LD2451::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txpin) {
    unsigned long startmillis = millis();
    //_radarSerial->setTimeout(500);
    //_radarSerial->setDebugOutput(true);
    _radarSerial->begin(baud, config, rxPin, txpin);
    xTaskCreatePinnedToCore(readerTask, "LD2451Reader", 4000, this, 1, &readerTaskHandle, xTaskGetCoreID(NULL));

    // one shot task to read current paramaters from the sensor
    // without a delay between cmds something gets lost
    xTaskCreate(initParamsTask, "InitParams", 3000, this, 1, &initParamsTaskHandle);

    // was trying to read parameters from the sensor here
    //  had to use larger then wanted delays between cmds to get it work
    // moved into initParamsTask which works a lot quicker
    /*if (_debugUart) _debugUart->println("enable config cmds");
    enableConfiguration();
    if (!waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT)) {
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    vTaskDelay(10);
    if (_debugUart) _debugUart->println("Read detection parameters");
    readDetectionParameters();
    vTaskDelay(10);
    if (_debugUart) _debugUart->println("Read sensitivity parameters");
    readSensitivityParameters();
    vTaskDelay(10);
    if (_debugUart) _debugUart->println("Read firmware version");
    readFirmwareVersion();
    vTaskDelay(10);
    if (_debugUart) _debugUart->println("Disable config cmds");
    endConfiguration();
    */
    
}

/**
 * private:  process the data buffer for ACKs
 * 
 * @param datalength: length of buffer
 */
void HLK_LD2451::processOther(uint16_t datalength, byte* data) {
    byte buffer[datalength];
    if (!usingBLE) {
        _radarSerial->readBytes(buffer, datalength);
    } else {
        memcpy(static_cast<void*>(buffer), static_cast<const void*>(data), datalength);
    }
    ackResult.result = static_cast<LD2451::ackResult>(buffer[1]);
    ackResult.cmd = static_cast<LD2451::cmdValue>(buffer[0]);
    if (_debugUart) _debugUart->print("ACK for ");
    switch(buffer[0]) {
        case LD2451::CMD_enableConfig: 
            if (_debugUart) _debugUart->print("Enable Configuration Commands : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            _InConfig = buffer[2]==0;
            break;
        case LD2451::CMD_endConfig: 
            if (_debugUart) _debugUart->print("Disable Configuration Commands : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            _InConfig = buffer[2] != 0;
            break;
        case LD2451::CMD_targetDetectionConfig: 
            if (_debugUart) _debugUart->print("Target detection parameters config : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        case LD2451::CMD_readTargetParameter: 
            if (_debugUart) _debugUart->print("Read target detection parameter : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            if (_debugUart) _debugUart->printf("Max Detection Distance : %dm\r\n", buffer[4]);
            this->configParameters.maxdistance = buffer[4];
            if (_debugUart) _debugUart->printf("Movement direction: %s\r\n", buffer[5]==0?"Away Only":buffer[5]==1?"Approach Only":"Detect All");
            this->configParameters.direction = buffer[5];
            if (_debugUart) _debugUart->printf("Min Speed: %dkm/h\r\n", buffer[6]);
            this->configParameters.minspeed = buffer[6];
            if (_debugUart) _debugUart->printf("No target delay time: %ds\r\n", buffer[7]);
            this->configParameters.delaytime = buffer[7];
            break;
        case LD2451::CMD_sensitivityConfig: 
            if (_debugUart) _debugUart->print("Radar sensitivity parameter configuration : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        case LD2451::CMD_readSensitivityConfig: 
            if (_debugUart) _debugUart->print("Radar sensitivity parameter query : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            if (_debugUart) _debugUart->printf("Cumulative effective trigger times : %d\r\n", buffer[4]);
            this->configParameters.triggertimes = buffer[4];
            if (_debugUart) _debugUart->printf("Signal-to-noise ratio threshold level (8Low - 0High): %d\r\n", buffer[5]);
            this->configParameters.snrthreshold = buffer[5];
            break;
        case LD2451::CMD_readFirmwareVersion: 
            if (_debugUart) _debugUart->print("Read firmware version : ");
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            firmware_data.type = buffer[4]<<8 | buffer[5];
            firmware_data.majora = buffer[7];
            firmware_data.majorb = buffer[6];
            firmware_data.minor = buffer[11]<<24 | buffer[10]<<16 | buffer[9]<<8 | buffer[8];
            //if (_debugUart) hexout(buffer+6, 6);
            if (_debugUart) _debugUart->printf("result: V%d.%02d.%x\r\n", buffer[7], buffer[6], buffer[11]<<24 | buffer[10]<<16 | buffer[9]<<8 | buffer[8]);
            break;
        case LD2451::CMD_setBaudrate: 
            if (_debugUart) _debugUart->print("Setting the serial port baud rate : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        case LD2451::CMD_restoreFactory: 
            if (_debugUart) _debugUart->print("Restoring factory settings : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        case LD2451::CMD_restartModule: 
            if (_debugUart) _debugUart->print("Restart module : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        default:
            if (_debugUart) _debugUart->println("Unknown ACK/Command");
    }
    xSemaphoreGive(ackWaiter);
}

/**
 * private: process the data buffer for detected targets
 *   each target is added to the end of the vector
 *   retrieve and empty vector using getTargets()
 * 
 * @param datalength: length of buffer
 */
void HLK_LD2451::processTargets(uint16_t datalength, byte* data) {
    
    byte buffer[datalength];
    if (!usingBLE) {
        _radarSerial->readBytes(buffer, datalength);
        if (_debugUart) {
            _debugUart->print("buff var: ");
            for(auto i=0; i<datalength; i++) {
                _debugUart->printf("%02x", buffer[i]);
                _debugUart->print(" ");
            }
            _debugUart->println();
        }
    } else {
        if (_debugUart) {
            _debugUart->print("data var: ");
            for(auto i=0; i<datalength; i++) {
                _debugUart->printf("%02x", data[i]);
                _debugUart->print(" ");
            }
            _debugUart->println();
        }
        memcpy(static_cast<void*>(buffer), static_cast<const void*>(data), datalength);
        if (_debugUart) {
            _debugUart->print("buff var: ");
            for(auto i=0; i<datalength; i++) {
                _debugUart->printf("%02x", buffer[i]);
                _debugUart->print(" ");
            }
            _debugUart->println();
        }
    }
    LD2451::vehicleTarget_t target;
    
    xSemaphoreTake(radarLock, portMAX_DELAY);
    for(auto numtargets = buffer[1]; numtargets>0; numtargets--) {
        if (_debugUart) _debugUart->printf("offset: %d\r\n", 2+(sizeof(LD2451::vehicleTarget_t)*(numtargets-1)));
        memcpy(static_cast<void *>(&target), static_cast<const void *>(buffer+2+(sizeof(LD2451::vehicleTarget_t)*(numtargets-1))), sizeof(LD2451::vehicleTarget_t));
        int16_t angleProcessing = *(uint8_t *)(&target.angle);
        if (_debugUart) Serial.printf("Angle: %d original - %d adjusted\r\n", angleProcessing, angleProcessing - 0x80);
        target.angle = angleProcessing-0x80;
        targets.push_back(target);
    }
    xSemaphoreGive(radarLock);
    if (buffer[1] > 0) xSemaphoreGive(detectionWaiter);
}

/**
 * private: task the continously reads the uart for target and ack data
 * 
 * @param param:  cast pointer to the owning class
 * 
 */
void HLK_LD2451::readerTask(void *param) {
    HLK_LD2451 *owner = static_cast<HLK_LD2451 *>(param);
    byte buffer[6];
    bool dummy;
    uint16_t datalength;
    uint8_t numTargets;
    LD2451::vehicleTarget_t target;
    size_t bytesread;
    int availableBytes = 0;
    if (owner->_debugUart) owner->_debugUart->printf("Reader Task Started on core %d\r\n", xPortGetCoreID());

    while(true) {
        availableBytes = owner->_radarSerial->available();
        while(availableBytes) {
            if (availableBytes >= 6) {
                
                // check for start marker && data length
                owner->_radarSerial->readBytes(buffer, 6);
                datalength = buffer[5] << 8 | buffer[4];

                // check for target data header
                if (buffer[0] == 0xf4 && buffer[1] == 0xf3 && buffer[2] == 0xf2 && buffer[3] == 0xf1) {
                    if (datalength) owner->processTargets(datalength);
                }

                // check for ack data header
                if (buffer[0] == 0xfd && buffer[1] == 0xfc && buffer[2] == 0xfb && buffer[3] == 0xfa) {
                    if (datalength) owner->processOther(datalength);
                }

                // empty buffer
                while(owner->_radarSerial->available()) {
                    int dummy = owner->_radarSerial->read();
                }
                
                availableBytes = owner->_radarSerial->available();
            } else {
                int dummy = owner->_radarSerial->read();
                availableBytes = owner->_radarSerial->available();
            }
        }
        
    }
}

/**
 * return a vector of detected targets
 *  then clears out the current list
 * 
 * @returns std::vector<LD2451::vehicleTarget_t>
 */
std::vector<LD2451::vehicleTarget_t> HLK_LD2451::getTargets(void) {
    xSemaphoreTake(radarLock, portMAX_DELAY);
    std::vector<LD2451::vehicleTarget_t> result = targets;
    targets.clear();
    xSemaphoreGive(radarLock);
    return result;
}

/**
 * send cmd to the sensor
 * 
 * @param command byte array of the command
 * @param length length of byte array
 */
size_t HLK_LD2451::sendCommand(uint8_t* command, size_t length) {
    uint8_t count=0;
    if (!usingBLE) {
        while(!_radarSerial->availableForWrite() && count < 100) {
            vTaskDelay(10);
            count++;
        }
        if (count == 100) return 0;
        if (_debugUart) _debugUart->println("Sending cmd via serial");
        return(_radarSerial->write(command, length));
    } else {
        if (_debugUart) _debugUart->println("Sending cmd via ble");
        bleCmd->writeValue(command, length, false);
        return length;
    }
}

/**
 * send Enabling Configuration Commands (1.2.1 in PDF)
 */
bool HLK_LD2451::enableConfiguration() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    return sizeof(command) == sendCommand(command, sizeof(command));
}

/**
 * send End configuration command (1.2.2 in PDF)
 */
bool HLK_LD2451::endConfiguration() {
    if (!_InConfig) return false;
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    return sizeof(command) == sendCommand(command, sizeof(command));

}

/**
 * send Target detection configuration commands (1.2.3 in PDF)
 * 
 * @param distance Maximum detection distance | 0x0A - 0xFF Unit: m
 * @param direction Movement direction setting | 00: away only | 01: approach only | 02: both away and approach
 * @param speed Minimum movement speed setting | 0x00 - 0x78 Unit: km/h
 * @param delay No target delay time setting | 0x00 - 0xFF Unit: s
 */
bool HLK_LD2451::setDetectionParameters(uint8_t distance, uint8_t direction, uint8_t speed, uint8_t delay) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x06, 0x00, 0x02, 0x00, distance, direction, speed, delay, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_targetDetectionConfig, HLK_LD2451_CMDACKWAIT);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Read target detection parameter command (1.2.4 in PDF)
 *   data is returned from sensor in the ACK - retrieved in processOther()
 *   once retrieved available via the class struct configParameters
 */
bool HLK_LD2451::readDetectionParameters() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x12, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_readTargetParameter, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_readTargetParameter, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Radar sensitivity parameter configuration command (1.2.5 in PDF)
 * 
 * @param trigCount Cumulative effective trigger times | 0x00 - 0xFF  The alarm information will be reported only when the number of consecutive detections is met.
 * @param snrLevel Signal-to-noise ratio threshold level | 00: The program default parameter is 4;  3-8: The larger the value, the lower the sensitivity and the more difficult it is to detect the target.
 * @param extParam1 (default value 0) Extended Parameters | 00 no information on what this does
 * @param extParam2 (default value 0) Extended Parameters | 00 no information on what this does
 */
bool HLK_LD2451::setSensitivityParameters(uint8_t trigCount, uint8_t snrLevel, uint8_t extParam1, uint8_t extParam2) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x06, 0x00, 0x03, 0x00, trigCount, snrLevel, extParam1, extParam2, 0x04, 0x03, 0x02, 0x01};

    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }

    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_sensitivityConfig, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_sensitivityConfig, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    if (result) { //
        
    }
    return result;
}

/**
 * send Radar sensitivity parameter query command (1.2.6 in PDF)
 *   data is returned from sensor in the ACK - retrieved in processOther()
 *   once retrieved available via the class struct configParameters
 */
bool HLK_LD2451::readSensitivityParameters() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x13, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_readSensitivityConfig, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_readSensitivityConfig, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Read firmware version command (1.2.7 in PDF)
 *   data is returned from sensor in the ACK - retrieved in processOther()
 *   once retrieved available via the class struct firmware_data or as a String via getFirmwareVersion()
 */
bool HLK_LD2451::readFirmwareVersion() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    if (sizeof(command) != sendCommand(command, sizeof(command))) return false;
    bool result = waitForAck(LD2451::CMD_readFirmwareVersion, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_readFirmwareVersion, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Restoring factory settings (1.2.9 in PDF)
 * This command is used to restore all configuration values to factory settings. The configuration values take effect after the module is restarted.
 */
bool HLK_LD2451::resetToFactory() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_restoreFactory, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_restoreFactory, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Restart module (1.2.10)
 * When the module receives this command, it will automatically restart after the response is sent.
 */
bool HLK_LD2451::rebootModule() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_restartModule, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_restartModule, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

/**
 * send Setting the serial port baud rate (1.2.8 in PDF)
 * This command is used to set the baud rate of the module serial port. The configuration value will not be lost when the power is off. The configuration value will take effect after the module is restarted.
 * 
 * @param baudrate  - use enum LD2451::baudRates to pass in baudrate selection value
 */
bool HLK_LD2451::setBaudRate(LD2451::baudRates baudrate) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA1, 0x00, baudrate, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint32_t currentBaudrate = _radarSerial->baudRate();
    uint32_t newBaudrate = 0;
    uint32_t baudrates[8] = { 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800 };
    
    bool setConfig = !_InConfig;
    newBaudrate = baudrates[baudrate-1];

    if (setConfig) {
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT);
    }
    sendCommand(command, sizeof(command));
    bool result =  waitForAck(LD2451::CMD_setBaudrate, HLK_LD2451_CMDACKWAIT);
    if (!result) {
        sendCommand(command, sizeof(command));
        result = waitForAck(LD2451::CMD_setBaudrate, HLK_LD2451_CMDACKWAIT);
    }
    if (setConfig) {
        endConfiguration();
    }
    
    return result;
}

/**
 * @returns String of the firmware version format (Vxx.xx.xxxxxx)
 */
String HLK_LD2451::getFirmwareVersion() {
    if (firmware_data.type == 0) {
        readFirmwareVersion();
    }
    char cver[100];
    sprintf(cver, "V%02x.%02x.%02x", firmware_data.majora, firmware_data.majorb, firmware_data.minor);
    String version = cver;
    return version;
}

/**
 * waits for an ACK from the sensor for cmd
 * 
 * @param cmd  byte value of cmd - enum LD2451::cmdValue is the valid ones
 * @param waitMillis how long to wait before timeing out
 * 
 * @returns true - ACK recevied   false - timedout
 */
bool HLK_LD2451::waitForAck(LD2451::cmdValue cmd, unsigned long waitMillis) {
    unsigned long waitingTime = waitMillis;
    unsigned long startTime = millis();
    if (_debugUart) _debugUart->printf("Waiting for ACK - (%d)%s\r\n", cmd, LD2451::cmdName(cmd).c_str());
    while (millis() < startTime + waitingTime) {
        if (pdTRUE == xSemaphoreTake(ackWaiter, waitingTime)) {
            if (ackResult.cmd == cmd) return ackResult.result == LD2451::ACK_Success;
            
            long timeDiff = millis() - startTime;
            if (timeDiff >= waitingTime) {
                if (_debugUart) _debugUart->println("Ack timeout");
                return false;
            }
            waitingTime = timeDiff;
            startTime = millis();
            if (_debugUart) _debugUart->println("Still Waiting for ACK");
        } else {
            if (_debugUart) _debugUart->println("Ack timeout");
            return false;
        }
    }
    if (_debugUart) _debugUart->println("Ack timeout");
    return false;
}

/**
 * waits for a detection event
 * 
 * @param timeout how long to wait before timeing out
 * 
 * @returns true - detection of vehicles   false - timedout/no vehicles detected
 */
bool HLK_LD2451::waitForDetection(unsigned long timeout) {
    return pdTRUE == xSemaphoreTake(detectionWaiter, timeout);
}

/**
 * private static
 *   Task the calls the read parameter cmds to populate the variables
 *   has to have a delay between each on or it just freezes
 */
void HLK_LD2451::initParamsTask(void * param) {
    HLK_LD2451 *owner = static_cast<HLK_LD2451 *>(param);
    constexpr unsigned long delayBetweenCmds = 10;
    Stream *debug = owner->_debugUart;
 
    if (debug) debug->printf("Init Params Task (IPT) on core %d\r\n", xPortGetCoreID());
    if (debug) debug->println("IPT: Enable config cmds");
    
    owner->enableConfiguration();
    owner->waitForAck(LD2451::CMD_enableConfig, HLK_LD2451_CMDACKWAIT*2);
    vTaskDelay(delayBetweenCmds);
    if (debug) debug->println("IPT: Read detection parameters");
    owner->readDetectionParameters();
    vTaskDelay(delayBetweenCmds);
    if (debug) debug->println("IPT: Read sensitivity parameters");
    owner->readSensitivityParameters();
    vTaskDelay(delayBetweenCmds);
    if (debug) debug->println("IPT: Read firmware version");
    owner->readFirmwareVersion();
    vTaskDelay(delayBetweenCmds);
    if (debug) debug->println("IPT: Disable config cmds");
    owner->endConfiguration();
    owner->waitForAck(LD2451::CMD_endConfig, HLK_LD2451_CMDACKWAIT*2);
    vTaskDelay(delayBetweenCmds);
    //if (debug) debug->printf("-----\r\nIPT: Min Stack Free: %d\r\n-----\r\n", uxTaskGetStackHighWaterMark(NULL));
    if (debug) debug->println("IPT: vTaskDelete");
    // don't need this task after one run
    vTaskDelete(NULL);
}