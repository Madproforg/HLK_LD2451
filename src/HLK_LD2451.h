#if !defined(ESP32)
#error HLK_LD2451 is written for an ESP32 variant only - uses of multitasking & semaphores
#endif

#if !defined(__HLK_LD2451_h__) 
#define __HLK_LD2451_h__

/**
 * pin config for uart / interrupt pin
 */
#if !defined(RADAR_TX)
    #define RADAR_TX 16
#endif

#if !defined(RADAR_RX) 
    #define RADAR_RX 15
#endif

#if !defined(RADAR_OT1)
    #define RADAR_OT1 17
#endif


#include <Arduino.h>
#include <vector>

#include <HardwareSerial.h>
#if defined(HLK2514ENABLE_BLE)        
#if !defined(USE_NIMBLE_LIBRARY)
#include <BLEDevice.h>
#else
#include <NimBLEDevice.h>
#endif
#endif
namespace LD2451 {
    /**
     * struct type for detected vehicle information
     * 
     */
    typedef struct  {
        int8_t angle; // degrees  (reported value -0x80 for actual)
        uint8_t distance; // metres
        uint8_t direction;  // 0 - away | 1 - towards  ?? might be reversed pdf table shows 0 away, example uses 0 towards
        uint8_t speed; // km/h
        uint8_t snr; // signale/noise 

    } vehicleTarget_t;

    /**
     * possible ACK results from sensor
     */
    enum ackResult {ACK_Success, ACK_Failure};

    /**
     * CMD values for using in waitForAck()
     */
    enum cmdValue { CMD_enableConfig = 0xff, 
                            CMD_endConfig = 0xFE,
                            CMD_targetDetectionConfig = 0x02,
                            CMD_readTargetParameter = 0x12,
                            CMD_sensitivityConfig = 0x03,
                            CMD_readSensitivityConfig = 0x13,
                            CMD_readFirmwareVersion = 0xA0,
                            CMD_setBaudrate = 0xA1,
                            CMD_restoreFactory = 0xA2,
                            CMD_restartModule = 0xA3
                        };

    /**
     * baudrates for setBaudRate
     */
    enum baudRates {
        BAUD_9600 = 1,
        BAUD_19200,
        BAUD_38400, 
        BAUD_57600,
        BAUD_115200, /* factory default */
        BAUD_230400,
        BAUD_256000,
        BAUD_460800
    };
}; // endnamespace

#if defined(HLK2514ENABLE_BLE)        
namespace HLK_LD2451BLE {
    class scanForLD2451Callbacks : public BLEAdvertisedDeviceCallbacks {
        void onResult(BLEAdvertisedDevice advertisedDevice);   
        BLEAddress* foundAddress = nullptr;
    };

}
#endif

class HLK_LD2451 {
    public:
        HLK_LD2451();
        HLK_LD2451(HardwareSerial *Uart);
        void debugOutput(Stream *debugUart);
#if defined(HLK2514ENABLE_BLE)        
        bool begin_BLE(BLEAddress *pAddress = nullptr);
#endif
        void begin(unsigned long baud=115200, uint32_t config=SERIAL_8N1, int8_t rxPin=RADAR_RX, int8_t txpin=RADAR_TX);
        std::vector<LD2451::vehicleTarget_t> getTargets(void);

        /**
         * struct containing firmware version
         */
        struct {
            uint16_t type;
            uint8_t majora;
            uint8_t majorb;
            uint32_t minor;
        } firmware_data;

        // struct contain sensor configuration parameters
        struct  {
            uint8_t maxdistance;
            uint8_t direction; // 0 away - 1 towards - 2 both
            uint8_t minspeed;
            uint8_t delaytime;
            uint8_t triggertimes;
            uint8_t snrthreshold; // 8 low - 0 high
        } configParameters;

        String getFirmwareVersion();
        bool enableConfiguration();
        bool endConfiguration();
        bool setDetectionParameters(uint8_t distance, uint8_t direction, uint8_t speed, uint8_t delay);
        bool readDetectionParameters();
        bool setSensitivityParameters(uint8_t trigCount, uint8_t snrLevel, uint8_t extParam1=0, uint8_t extParam2=0);
        bool readSensitivityParameters();
        bool readFirmwareVersion();
        bool resetToFactory();
        bool rebootModule();
        bool waitForAck(LD2451::cmdValue cmd, unsigned long waitMillis);
        bool setBaudRate(LD2451::baudRates baudrate);
        bool waitForDetection(unsigned long timeout);
#if defined(HLK2514ENABLE_BLE)        
        void processBLEData(uint8_t* pData, size_t length);
#endif
    private :
        // uart for talking to sensor
        HardwareSerial* _radarSerial;
        // handle for the task that reads from the sensor
        TaskHandle_t readerTaskHandle, initParamsTaskHandle;

        // pointer to a uart for ouputing debugging info
        Stream *_debugUart;
        
        SemaphoreHandle_t radarLock;
        SemaphoreHandle_t ackWaiter;
        SemaphoreHandle_t detectionWaiter;
        std::vector<LD2451::vehicleTarget_t> targets;
        bool _InConfig = false;

        struct {
            LD2451::ackResult result;
            LD2451::cmdValue cmd;

        } ackResult;
        bool usingBLE = false;
        static void readerTask(void *param);
        static void initParamsTask(void *param);
        
        void processOther(uint16_t datalength, byte* data=nullptr);
        void processTargets(uint16_t datalength, byte* data=nullptr);
        size_t sendCommand(uint8_t* command, size_t length);

#if defined(HLK2514ENABLE_BLE)        
        // ble vars
        static void newData(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
        BLEAddress *bleServerAddress; // ble mac address of sensor
        BLEClient* pClient;
        BLERemoteService* pRemoteService;
        BLERemoteCharacteristic* bleData;
        BLERemoteCharacteristic* bleCmd;
        BLERemoteService* sensorRemoteService;
#endif

};
#endif