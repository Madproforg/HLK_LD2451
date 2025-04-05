#if !defined(ESP32)
#error HLK_LD2451 is written for an ESP32 variant only - uses multi tasking and semaphores
#endif
/**
 * starting point for this code
 *  posts on the arduino forum https://forum.arduino.cc/t/using-hlk-ld2451-radar-for-speed-measurment/1328105
 * and the ld2410 library https://github.com/ncmreynolds/ld2410
 * 
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include "HLK_LD2451.h"

void hexout(byte *buffer, int len) {
    for (auto x=0; x<len; x++) {
        Serial.printf("%02X", *(buffer+x));
    }
}

namespace LD2451 {
    String cmdName(LD2451::cmdValue cmd) {
        switch (cmd) {
            case CMD_enableConfig: return String("Enable Config");
            case CMD_endConfig: return String("End Config");
            case CMD_targetDetectionConfig: return String("Target Detection Config");
            case CMD_readFirmwareVersion: return String("Read Firmware Version");
            case CMD_readSensitivityConfig: return String("Read Sensitivity Config");
            case CMD_readTargetParameter: return String("Read Target Parameter");
            case CMD_restartModule: return String("Restart Module");
            case CMD_restoreFactory: return String("Factory Reset");
            case CMD_sensitivityConfig: return String("Sensitivity Config");
            case CMD_setBaudrate: return String("Set Baud Rate");
        }
        return String("Unknown CMD: ") + String(cmd, 16);
    }
}// end namespace

HLK_LD2451::HLK_LD2451(HardwareSerial &Uart) : _radarSerial(Uart) {
    radarLock = xSemaphoreCreateMutex();
    ackWaiter = xSemaphoreCreateBinary();
    detectionWaiter = xSemaphoreCreateBinary();
    _debugUart = NULL;
    firmware_data.type == 0;
}

void HLK_LD2451::debugOutput(Stream *uart) {
    _debugUart = uart;
}

void HLK_LD2451::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txpin) {
    unsigned long startmillis = millis();
    _radarSerial.begin(baud, config, rxPin, txpin);
    xTaskCreatePinnedToCore(readerTask, "LD2451Reader", 4000, this, 1, &readerTaskHandle, xTaskGetCoreID(NULL));
    while(millis() - startmillis < 5000) {
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 500);
        if (_InConfig) break;
        yield();
    }

    readDetectionParameters();
    readSensitivityParameters();
    getFirmwareVersion();
    endConfiguration();
}

void HLK_LD2451::processOther(uint16_t datalength) {
    byte buffer[datalength];
    int numbytes = _radarSerial.readBytes(buffer, datalength);
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
            this->_configParameters.maxdistance = buffer[4];
            if (_debugUart) _debugUart->printf("Movement direction: %s\r\n", buffer[5]==0?"Away Only":buffer[5]==1?"Approach Only":"Detect All");
            this->_configParameters.direction = buffer[5];
            if (_debugUart) _debugUart->printf("Min Speed: %dkm/h\r\n", buffer[6]);
            this->_configParameters.minspeed = buffer[6];
            if (_debugUart) _debugUart->printf("No target delay time: %ds\r\n", buffer[7]);
            this->_configParameters.delaytime = buffer[7];
            break;
        case LD2451::CMD_sensitivityConfig: 
            if (_debugUart) _debugUart->print("Radar sensitivity parameter configuration : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            break;
        case LD2451::CMD_readSensitivityConfig: 
            if (_debugUart) _debugUart->print("Radar sensitivity parameter query : "); 
            if (_debugUart) _debugUart->println(buffer[2]==0?"Success":"Failure");
            if (_debugUart) _debugUart->printf("Cumulative effective trigger times : %d\r\n", buffer[4]);
            this->_configParameters.triggertimes = buffer[4];
            if (_debugUart) _debugUart->printf("Signal-to-noise ratio threshold level (8Low - 0High): %d\r\n", buffer[5]);
            this->_configParameters.snrthreshold = buffer[5];
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

void HLK_LD2451::processTargets(uint16_t datalength) {
    byte buffer[datalength];
    _radarSerial.readBytes(buffer, datalength);
    LD2451::vehicleTarget_t target;
    
    xSemaphoreTake(radarLock, portMAX_DELAY);
    for(auto numtargets = buffer[1]; numtargets>0; numtargets--) {
        //if (_debugUart) hexout(buffer+2, datalength-1);
        memcpy(static_cast<void *>(&target), static_cast<const void *>(buffer+2), sizeof(LD2451::vehicleTarget_t));
        int16_t angleProcessing = *(uint8_t *)(&target.angle);
        if (_debugUart) Serial.printf("Angle: %d original - %d adjusted\r\n", angleProcessing, angleProcessing - 0x80);
        target.angle = angleProcessing-0x80;
        targets.push_back(target);
    }
    if (buffer[1] > 0) xSemaphoreGive(detectionWaiter);
    xSemaphoreGive(radarLock);
}

void HLK_LD2451::readerTask(void *param) {
    HLK_LD2451 *owner = static_cast<HLK_LD2451 *>(param);
    byte buffer[6];
    bool dummy;
    uint16_t datalength;
    uint8_t numTargets;
    LD2451::vehicleTarget_t target;
    size_t bytesread;
    int availableBytes = 0;
    if (owner->_debugUart) owner->_debugUart->println("Reader Task Started");


    while(true) {
        availableBytes = owner->_radarSerial.available();
        while(availableBytes) {
            if (availableBytes >= 6) {
                
                // check for start marker && data length
                owner->_radarSerial.readBytes(buffer, 6);
                datalength = buffer[5] << 8 | buffer[4];
                //hexout(buffer+4,2); 
                //Serial.println("");
                if (buffer[0] == 0xf4 && buffer[1] == 0xf3 && buffer[2] == 0xf2 && buffer[3] == 0xf1) {
                    if (datalength) owner->processTargets(datalength);
                }
                if (buffer[0] == 0xfd && buffer[1] == 0xfc && buffer[2] == 0xfb && buffer[3] == 0xfa) {
                    if (datalength) owner->processOther(datalength);
                }
                // empty buffer
                while(owner->_radarSerial.available()) {
                    int dummy = owner->_radarSerial.read();
                }
                //radarSerial.readBytes(buffer, 4);
                availableBytes = owner->_radarSerial.available();
            } else {
                int dummy = owner->_radarSerial.read();
                availableBytes = owner->_radarSerial.available();
            }
        }
        
    }
}

std::vector<LD2451::vehicleTarget_t> HLK_LD2451::getTargets(void) {
    xSemaphoreTake(radarLock, portMAX_DELAY);
    std::vector<LD2451::vehicleTarget_t> result = targets;
    targets.clear();
    xSemaphoreGive(radarLock);
    return result;
}

size_t HLK_LD2451::sendCommand(uint8_t* command, size_t length) {
    return(_radarSerial.write(command, length));
}

bool HLK_LD2451::enableConfiguration() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    return sizeof(command) == sendCommand(command, sizeof(command));
}
bool HLK_LD2451::endConfiguration() {
    if (!_InConfig) return false;
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    return sizeof(command) == sendCommand(command, sizeof(command));

}

bool HLK_LD2451::setDetectionParameters(uint8_t distance, uint8_t direction, uint8_t speed, uint8_t delay) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x06, 0x00, 0x02, 0x00, distance, direction, speed, delay, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_targetDetectionConfig, 100);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

bool HLK_LD2451::readDetectionParameters() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x12, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_readTargetParameter, 100);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

bool HLK_LD2451::setSensitivityParameters(uint8_t trigCount, uint8_t snrLevel, uint8_t extParam1, uint8_t extParam2) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x06, 0x00, 0x03, 0x00, trigCount, snrLevel, extParam1, extParam2, 0x04, 0x03, 0x02, 0x01};

    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }

    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_sensitivityConfig, 100);
    if (setConfig) { 
        endConfiguration();
    }
    if (result) { //
        
    }
    return result;
}

bool HLK_LD2451::readSensitivityParameters() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x13, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_readSensitivityConfig, 100);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

bool HLK_LD2451::readFirmwareVersion() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_readFirmwareVersion, 100);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

bool HLK_LD2451::resetToFactory() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_restoreFactory, 100);
    if (setConfig) { 
        endConfiguration();
    }
    _radarSerial.updateBaudRate(115200);
    return result;
}

bool HLK_LD2451::rebootModule() {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
    bool setConfig = !_InConfig;
    if (setConfig) { 
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result = waitForAck(LD2451::CMD_restartModule, 100);
    if (setConfig) { 
        endConfiguration();
    }
    return result;
}

bool HLK_LD2451::setBaudRate(LD2451::baudRates baudrate) {
    uint8_t command[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA1, 0x00, baudrate, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint32_t currentBaudrate = _radarSerial.baudRate();
    uint32_t newBaudrate = 0;
    uint32_t baudrates[8] = { 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800 };
    
    bool setConfig = !_InConfig;
    newBaudrate = baudrates[baudrate-1];

    if (setConfig) {
        enableConfiguration();
        waitForAck(LD2451::CMD_enableConfig, 50);
    }
    sendCommand(command, sizeof(command));
    bool result =  waitForAck(LD2451::CMD_setBaudrate, 100);
    if (setConfig) {
        endConfiguration();
    }
    _radarSerial.updateBaudRate(newBaudrate);
    return result;
}

String HLK_LD2451::getFirmwareVersion() {
    if (firmware_data.type == 0) {
        readFirmwareVersion();
    }
    char cver[100];
    sprintf(cver, "V%02x.%02x.%02x", firmware_data.majora, firmware_data.majorb, firmware_data.minor);
    String version = cver;
    return version;
}

bool HLK_LD2451::waitForAck(LD2451::cmdValue cmd, unsigned long waitMillis) {
    unsigned long waitingTime = waitMillis;
    unsigned long startTime = millis();
    if (_debugUart) _debugUart->printf("Waiting for ACK - (%d)%s\r\n", cmd, LD2451::cmdName(cmd));
    while (millis() < startTime + waitingTime) {
        if (pdTRUE == xSemaphoreTake(ackWaiter, waitingTime)) {
            if (ackResult.cmd == cmd) return ackResult.result == LD2451::ACK_Success;
            
            long timeDiff = millis() - startTime;
            if (timeDiff >= waitingTime) return false;
            waitingTime = timeDiff;
            startTime = millis();
            if (_debugUart) _debugUart->println("Still Waiting for ACK");
        } else {
            return false;
        }
    }
    return false;
}

bool HLK_LD2451::waitForDetection(unsigned long timeout) {
    return pdTRUE == xSemaphoreTake(detectionWaiter, timeout);
}