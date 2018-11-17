#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SimpleKalmanFilter.h>
#undef B1

// Connection of BMP085
// SDA - pin 21
// SCL - pin 22
// EOC - pin 23

constexpr uint8_t SENSOR_SPI_ADDR = 0x77;
constexpr uint8_t OSS = 3;
constexpr uint8_t EOC_PIN = 23;

constexpr char* BLE_NAME = "SENSOR";
constexpr char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char* CHARACTERISTIC_UUID = "df74a24a-be00-495a-9f5e-b8d87aaba650";

SimpleKalmanFilter g_kalman_filter(1, 1, 0.01);
BLECharacteristic *g_BLE_data;
volatile bool g_value_ready = false;
volatile bool g_device_connected = false;

struct BMP085Controller {
    uint8_t _sensor_spi_address;
    uint8_t _oss;
    uint8_t _eoc_pin;

    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int32_t B5;
    int16_t MB;
    int16_t MC;
    int16_t MD;

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

      T result = 0;
      for (uint8_t i = 0; i != SIZE; ++i) {
        result = result << 8;
        result |= Wire.read();
      }

      return result;
    }

  public:
    BMP085Controller(uint8_t eoc_pin, uint8_t sensor_spi_address, uint8_t oss):
      _eoc_pin(eoc_pin), _sensor_spi_address(sensor_spi_address), _oss(oss)
    {
      AC1 = sensorRead<int16_t>(0xAA);
      AC2 = sensorRead<int16_t>(0xAC);
      AC3 = sensorRead<int16_t>(0xAE);
      AC4 = sensorRead<uint16_t>(0xB0);
      AC5 = sensorRead<uint16_t>(0xB2);
      AC6 = sensorRead<uint16_t>(0xB4);
      B1 = sensorRead<int16_t>(0xB6);
      B2 = sensorRead<int16_t>(0xB8);
      MB = sensorRead<int16_t>(0xBA);
      MC = sensorRead<int16_t>(0xBC);
      MD = sensorRead<int16_t>(0xBE);
    }

    void requestTemperature() {
      sensorWrite(0xF4, 0x2E);
    }

    void requestPressure() {
      sensorWrite(0xF4, 0x34 + (_oss << 6));
    }

    int32_t readTemperature() {
      int32_t UT = sensorRead<uint16_t>(0xF6);

      int32_t X1 = (UT - (int32_t)AC6) * ((int32_t)AC5) >> 15;
      int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);
      B5 = X1 + X2;
      return (B5 + 8) >> 4;
    }

    int32_t readPressure() {
      int32_t UP, X1, X2, X3, B3, B6, p;
      uint32_t B4, B7;

      UP = sensorRead<int32_t, 3>(0xF6) >> (8 - _oss);
      B6 = B5 - 4000;
      X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
      X2 = (AC2 * B6) >> 11;
      X3 = X1 + X2;
      B3 = (((((int32_t)AC1) * 4 + X3) << _oss) + 2) >> 2;
      X1 = (AC3 * B6) >> 13;
      X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
      X3 = ((X1 + X2) + 2 ) >> 2;
      B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
      B7 = ((uint32_t)(UP - B3) * (50000 >> _oss));
      p = (B7 < 0x80000000) ? (B7 << 1) / B4 : (B7 / B4) << 1;
      X1 = (p >> 8) * (p >> 8);
      X1 = (X1 * 3038) >> 16;
      X2 = (-7357 * p) >> 16;
      p += (X1 + X2 + 3791) >> 4;

      return p;
    }
} *controller;

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
  Serial.begin(9600);
  Wire.begin();

  controller = new BMP085Controller(EOC_PIN, SENSOR_SPI_ADDR, OSS);

  BLEDevice::init(BLE_NAME);
  BLEServer* BLE_server = BLEDevice::createServer();
  BLE_server->setCallbacks(new MyServerCallbacks());
  BLEService* BLE_service = BLE_server->createService(SERVICE_UUID);

  g_BLE_data = BLE_service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_READ);
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

    if (++counter == (1 << 4)) {
      data.temp = temp_sum >> 4;
      data.press = g_kalman_filter.updateEstimate(press_sum >> 4);
      Serial.println(data.temp);
      Serial.println(data.press);

      g_BLE_data->setValue((uint8_t*)&data, sizeof(data));
      g_BLE_data->notify();

      counter = temp_sum = press_sum = 0;
    }
  }
}
