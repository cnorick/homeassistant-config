#include "esphome.h"
#define STATUS_CMD 0x71
#define ADDR 0x38
#define COMPONENT_NAME "dht20"

// Based on DHT20 dataseet: https://cdn.sparkfun.com/assets/8/a/1/5/0/DHT20.pdf.
class DHT20 : public PollingComponent {
 public:
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();

  DHT20(int pollingRate) : PollingComponent(pollingRate) { }

  void setup() override {
    delay(100);
    checkStatus();
    delay(10); // Need to wait this long before requesting measurement.
  }

  float get_setup_priority() const override { return esphome::setup_priority::BUS; }

  void update() override {
    Wire.beginTransmission(ADDR);

    triggerMeasurement();
    char* data = readMeasurement();
    double humidity = calculateHumidity(data);
    double temperature = calculateTemperature(data);
    if (isDataError(data) ) {
      delete data;
      return;
    }
    delete data;

    //ESP_LOGD(COMPONENT_NAME, "Temperature: %f, Humidity: %f", temperature, humidity);
    temperature_sensor->publish_state(temperature);
    humidity_sensor->publish_state(humidity);

    Wire.endTransmission();
  }


  private:
  char* getBytes(int numBytes) {
    char* bytes = new char[numBytes];
    Wire.requestFrom(ADDR, numBytes);
    for (int i = 0; i < numBytes && Wire.available(); i++) {
      bytes[i] = Wire.read();
    }
    return bytes;
  }

  double calculateHumidity(char* data) {
    uint firstByte = data[1]; // Skip state byte.
    uint secondByte = data[2];
    uint thridByte = data[3];
    uint h = (firstByte << 12) | (secondByte << 4) | (thridByte >> 4);
    //ESP_LOGD(COMPONENT_NAME, "0X%X", h);

    return (double)h / (double)(1L << 20) * 100.0;
  }

  double calculateTemperature(char* data) {
    uint firstByte = data[3]; // Skip state byte and humidity bytes.
    uint secondByte = data[4];
    uint thridByte = data[5];
    uint t = ((firstByte & 0x0F) << 16) | (secondByte << 8) | thridByte;
    //ESP_LOGD(COMPONENT_NAME, "0X%X", t);

    return (double)t / (double)(1L << 20) * 200.0 - 50.0;
  }

  void checkStatus() {
    Wire.write(STATUS_CMD);
    char* status = getBytes(1);
    //logBytes(status, 1);
    char expectedStatus = 0x18;
    if ((*status & expectedStatus) != expectedStatus) {
      ESP_LOGE(COMPONENT_NAME, "Registers need to be initialized. See https://cdn.sparkfun.com/assets/8/a/1/5/0/DHT20.pdf for more information");
    }
    delete status;
  }

  void triggerMeasurement()
  {
    Wire.write(0xAC);
    Wire.write(0x33);
    Wire.write(0x00);
  }

  bool isBitSet(char c, int n) {
    return (c >> n) & 1U;
  }

  void waitUntilMeasurementIsReady() {
    while(true) {
      delay(80);
      char* status = getBytes(1);
      //logBytes(status, 1);
      if (isBitSet(*status, 7)) {
        ESP_LOGD(COMPONENT_NAME, "Measurement wasn't ready. Trying again...");
        continue;
      }
      delete status;
      return;
    }
  }

  char* readMeasurement()
  {
    waitUntilMeasurementIsReady();

    char* data = getBytes(6);
    //logBytes(data, 6);
    return data;
  }

  // TODO: Implement crc check.
  bool isDataError(char* data) {
    //uint x = (data[0] << 40) || (data[1] << 32) || (data[2] << 24) || (data[3] << 16) || (data[4] << 8) || data[5];
    //uint crc = 1 + x^4 + x^5 + x^8;
    //uint32_t crc = gencrc(data, 6);
    //char* value = getBytes(1);
    //ESP_LOGD(COMPONENT_NAME, "%d ", crc == (uint)*value);
    //ESP_LOGD(COMPONENT_NAME, "%d == %d ", crc, *value);
    //delete value;
    return false;
  }

  uint8_t gencrc(char* data, size_t len) {
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

  void logBytes(char* c, int numBytes) {
    char* s = new char[numBytes * 5];
    for (int i = 0; i < numBytes; i++) {
      char temp[6];
      sprintf(temp, "0x%X ", *(c + i));
      strncpy(s + i*5, temp, 5);
    }
    s[numBytes * 5 - 1] = '\0'; // Make the last space a terminator.
    ESP_LOGD(COMPONENT_NAME, "%s ", s);
    delete s;
  }
};