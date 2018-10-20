#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#undef B1

constexpr uint8_t SENSOR_SPI_ADDR = 0x77;
constexpr uint8_t OSS = 3;
constexpr uint8_t EOC_PIN = 23;
constexpr uint8_t POWER_PIN = 22;

constexpr char* BLE_NAME = "SENSOR";
constexpr char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char* UUID_TEMP = "df74a24a-be00-495a-9f5e-b8d87aaba650";
constexpr char* UUID_PRESS = "74657e0f-d97a-4b44-83a0-99f241dd9848";

BLECharacteristic *g_BLE_data;
volatile bool g_value_ready = false;

struct BMP085Controller {
    uint8_t _sensor_spi_address;
    uint8_t _oss;
    uint8_t _eoc_pin;
    uint8_t _power_pin;

    struct {
      int16_t AC1;
      int16_t AC2;
      int16_t AC3;
      uint16_t AC4;
      uint16_t AC5;
      uint16_t AC6;
      int16_t B1;
      int16_t B2;
      int16_t MB;
      int16_t MC;
      int16_t MD;
    } _calibration_coeficients;

    void sensorWrite(uint8_t address, uint8_t value) const {
      Wire.beginTransmission(_sensor_spi_address);
      Wire.write(address);
      Wire.write(value);
      Wire.endTransmission();
    } 
    
    template <typename T, size_t SIZE = sizeof(T)>
    T sensorRead(uint8_t address) const {
      Wire.beginTransmission(_sensor_spi_address);
      Wire.write(address);
      Wire.endTransmission();
      
      Wire.requestFrom(_sensor_spi_address, SIZE);
      while(Wire.available() < SIZE);
      
      T result = 0;
      for (uint8_t i = 0; i != SIZE; ++i) {
        result = result << 8;
        result |= Wire.read();
      }

      return result;
    }
  
  public:
    void enable() {
      digitalWrite(_power_pin, LOW);
      delay(100);
    }

    void disable() {
      digitalWrite(_power_pin, HIGH);
      delay(100);
    }
    
    BMP085Controller(uint8_t power_pin, uint8_t eoc_pin, uint8_t sensor_spi_address, uint8_t oss):
      _power_pin(power_pin), _eoc_pin(eoc_pin), _sensor_spi_address(sensor_spi_address), _oss(oss)    
    {
      enable();
      _calibration_coeficients.AC1 = sensorRead<int16_t>(0xAA);
      _calibration_coeficients.AC2 = sensorRead<int16_t>(0xAC);
      _calibration_coeficients.AC3 = sensorRead<int16_t>(0xAE);
      _calibration_coeficients.AC4 = sensorRead<uint16_t>(0xB0);
      _calibration_coeficients.AC5 = sensorRead<uint16_t>(0xB2);
      _calibration_coeficients.AC6 = sensorRead<uint16_t>(0xB4);
      _calibration_coeficients.B1 = sensorRead<int16_t>(0xB6);
      _calibration_coeficients.B2 = sensorRead<int16_t>(0xB8);
      _calibration_coeficients.MB = sensorRead<int16_t>(0xBA);
      _calibration_coeficients.MC = sensorRead<int16_t>(0xBC);
      _calibration_coeficients.MD = sensorRead<int16_t>(0xBE);
      disable();
    }

    void requestTemperature() {
      sensorWrite(0xF4, 0x2E);  
    }

    int32_t B5 = 0;
    int32_t readTemperature() {
      int32_t UP = sensorRead<uint16_t>(0xF6);
      int32_t X1 = ((UP - _calibration_coeficients.AC6) * _calibration_coeficients.AC5) >> 15;
      int32_t X2 = ((int32_t)_calibration_coeficients.MC << 11) / (X1 + _calibration_coeficients.MD);
      B5 = X1 + X2; 
      return (B5 + 8) >> 4; 
    }
    
    /*int32_t compensateTemperature(int32_t temp) {
      int32_t X1 = ((temp - _calibration_coeficients.AC6) * _calibration_coeficients.AC5) >> 15;
      int32_t X2 = ((int32_t)_calibration_coeficients.MC << 11) / (X1 + _calibration_coeficients.MD);
      B5 = X1 + X2;

      return (B5 + 8) >> 4; 
    }*/

    int32_t compensatePressure(int32_t press) {
      return press; 
    }

    void requestPressure() {
      sensorWrite(0xF4, 0x34 + (_oss<<6));  
    }

    uint32_t readPressure() {
      return sensorRead<uint32_t, 3>(0xF6);
    }

    uint16_t getTemperature() {
      requestTemperature();
      while (digitalRead(_eoc_pin) != HIGH);
      return readTemperature();
    }
    
    uint32_t getPressure() {
      requestPressure();
      while (digitalRead(_eoc_pin) != HIGH);
      return readPressure();
    }
} *controller;

volatile bool g_device_connected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* BLE_server) {
      g_device_connected = true;
    };

    void onDisconnect(BLEServer* BLE_server) {
      g_device_connected = false;
      BLE_server->getAdvertising()->start();
    }
};

void setup() {
  pinMode(POWER_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  
  controller = new BMP085Controller(POWER_PIN, EOC_PIN, SENSOR_SPI_ADDR, OSS);
  Serial.println(controller->_calibration_coeficients.AC5);
  
  BLEDevice::init(BLE_NAME);
  BLEServer* BLE_server = BLEDevice::createServer();
  BLE_server->setCallbacks(new MyServerCallbacks());
  BLEService* BLE_service = BLE_server->createService(SERVICE_UUID);
  
  g_BLE_data = BLE_service->createCharacteristic(UUID_TEMP, BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_READ);
  g_BLE_data->addDescriptor(new BLE2902());

  BLE_service->start();
  BLE_server->getAdvertising()->start();

  attachInterrupt(digitalPinToInterrupt(EOC_PIN), []{g_value_ready = true;}, RISING);
}

void loop() {
  static size_t counter = 0;
  static int32_t temp_sum = 0;
  static int32_t press_sum = 0;
  struct {
      int32_t temp;
      int32_t press;
  } data;

  if (g_device_connected) {
    controller->requestTemperature();
    while (g_device_connected && !g_value_ready);
    g_value_ready = false;
    temp_sum += controller->readTemperature();
    
    controller->requestPressure();
    while (g_device_connected && !g_value_ready);
    g_value_ready = false;
    press_sum += controller->readPressure();
    
    if (counter++ == 16) {
      data.temp = temp_sum >> 4;
      data.press = controller->compensatePressure(press_sum >> 4);
      Serial.println(data.temp);
      Serial.println(data.press);
      
      g_BLE_data->setValue((uint8_t*)&data, sizeof(data));
      g_BLE_data->notify();
      
      counter = temp_sum = press_sum = 0;
    }
  }
}
