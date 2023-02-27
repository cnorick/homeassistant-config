#include "esphome.h"
#include <cstring>
#include "SevSeg.h"
#define COMPONENT_NAME "7seg"

class SevenSegmentDisplay : public Component {
  public:

  // segMode = 0 (common cathode), = 1 (common anode)
  SevenSegmentDisplay(int numDigits, byte *digitPins, byte* segmentPins)
  : Component() {
    _digitPins = digitPins;
    _segmentPins = segmentPins;
    _numDigits = numDigits;
    _screenOn = false;

    bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
    byte hardwareConfig = COMMON_CATHODE; // See README.md for options
    bool updateWithDelays = false; // Default 'false' is Recommended
    bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
    bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

    sevseg.begin(hardwareConfig, _numDigits, _digitPins, _segmentPins, resistorsOnSegments,
      updateWithDelays, leadingZeros, disableDecPoint);
  }

  void setup() override { }

  void loop() override {
    if (_screenOn) {
      sevseg.refreshDisplay();
    }
  }

  void turnOff() {
    _screenOn = false;
    sevseg.blank();
  }

  void turnOn() {
    _screenOn = true;
  }

  void showBothTemps(float currentTemp, float setTemp) {
    char buf[6];
    int currentTempF = (int)cToF(currentTemp);
    int setTempF = (int)cToF(setTemp);

    if (setTempF > 99 && currentTempF > 99) {
      sprintf(buf, "hi.hi");
    }
    else if (setTempF > 99) {
      sprintf(buf, "%d.hi", currentTempF);
    }
    else if (currentTempF > 99) {
      sprintf(buf, "hi.%d", setTempF);
    }
    else {
      sprintf(buf, "%d.%d", currentTempF, setTempF);
    }

    sevseg.setChars(buf);
  }

  private:
  float cToF(float cTemp) {
    return cTemp * (9.0/5.0) + 32.0;
  }

  SevSeg sevseg;
  int _numDigits;
  byte* _digitPins;
  byte* _segmentPins;
  bool _screenOn;

};