#ifndef PARTY_CONTROLLER_H
#define PARTY_CONTROLLER_H

#include <Arduino.h>
#include "LEDController.h"

class PartyController {
public:
  explicit PartyController(LEDController &led);
  void begin();
  void start();            // called when entering PARTY mode
  void stop();             // called when leaving PARTY mode
  void update();           // call repeatedly from loop() while in PARTY
  void setSpeed(float degPerSec); // hue degrees per second
  void setSaturation(float s);    // 0..1
  void setBrightness(float v);    // 0..1

  // Smooth fade time in milliseconds (smaller = snappier, larger = smoother)
  void setFadeMs(float ms);

private:
  LEDController &led;
  unsigned long lastMillis;
  float hue;               // 0..360
  float speed;             // degrees per second
  float saturation;        // 0..1
  float brightness;        // 0..1

  // smoothing / fade state
  float fadeMs;            // time constant for smoothing in ms
  float currentR;          // current (smoothed) PWM values, 0..PWM_MAX
  float currentG;
  float currentB;

  void hsvToRgb(float h, float s, float v, uint16_t &r, uint16_t &g, uint16_t &b);
};

#endif // PARTY_CONTROLLER_H