#include "PartyController.h"
#include <math.h> // expf, fmod, fabsf

static const uint16_t PWM_MAX = 2047; // match your project's PWM range

PartyController::PartyController(LEDController &led)
  : led(led),
    lastMillis(0),
    hue(0.0f),
    speed(120.0f),
    saturation(1.0f),
    brightness(1.0f),
    fadeMs(150.0f),        // default fade time = 150 ms (tune as desired)
    currentR(0.0f),
    currentG(0.0f),
    currentB(0.0f)
{
}

void PartyController::begin()
{
  lastMillis = millis();
}

void PartyController::start()
{
  hue = 0.0f;
  lastMillis = millis();

  // initialize current RGB to the initial target so we don't jump abruptly on entry
  uint16_t r, g, b;
  hsvToRgb(hue, saturation, brightness, r, g, b);
  currentR = (float)r;
  currentG = (float)g;
  currentB = (float)b;
}

void PartyController::stop()
{
  // optionally fade out quickly or immediately turn off.
  // We'll immediately turn off here; if you want a fade-out, set current* to 0 and let update() drive it.
  led.setPWMDirectly(0, 0, 0);
}

void PartyController::setSpeed(float degPerSec) { speed = degPerSec; }
void PartyController::setSaturation(float s)     { saturation = constrain(s, 0.0f, 1.0f); }
void PartyController::setBrightness(float v)     { brightness = constrain(v, 0.0f, 1.0f); }
void PartyController::setFadeMs(float ms)        { fadeMs = max(1.0f, ms); } // avoid zero

void PartyController::update()
{
  unsigned long now = millis();
  unsigned long dt = (now - lastMillis);
  if (dt == 0) return;
  lastMillis = now;

  // advance hue (degrees)
  hue += (speed * (dt / 1000.0f));
  if (hue >= 360.0f) hue -= 360.0f;
  if (hue < 0.0f) hue += 360.0f;

  // compute target RGB for this hue/saturation/brightness
  uint16_t tr, tg, tb;
  hsvToRgb(hue, saturation, brightness, tr, tg, tb);

  // Convert targets to float PWM domain
  float targetR = (float)tr;
  float targetG = (float)tg;
  float targetB = (float)tb;

  // Exponential smoothing factor alpha = 1 - exp(-dt / tau)
  // where tau = fadeMs. This gives smooth, time-constant based smoothing.
  float alpha = 1.0f - expf(- (float)dt / (fadeMs > 0.0f ? fadeMs : 1.0f));
  // clamp alpha to sensible range
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  // apply smoothing
  currentR += (targetR - currentR) * alpha;
  currentG += (targetG - currentG) * alpha;
  currentB += (targetB - currentB) * alpha;

  // write smoothed PWM values
  led.setPWMDirectly((int)constrain((int)currentR, 0, PWM_MAX),
                     (int)constrain((int)currentG, 0, PWM_MAX),
                     (int)constrain((int)currentB, 0, PWM_MAX));
}

// basic HSV->RGB conversion, result scaled to 0..PWM_MAX
void PartyController::hsvToRgb(float h, float s, float v, uint16_t &outR, uint16_t &outG, uint16_t &outB)
{
  float c = v * s;
  float hh = fmod(h, 360.0f) / 60.0f;
  float x = c * (1.0f - fabsf(fmod(hh, 2.0f) - 1.0f));
  float r1 = 0, g1 = 0, b1 = 0;
  if (0.0f <= hh && hh < 1.0f) { r1 = c; g1 = x; b1 = 0; }
  else if (1.0f <= hh && hh < 2.0f) { r1 = x; g1 = c; b1 = 0; }
  else if (2.0f <= hh && hh < 3.0f) { r1 = 0; g1 = c; b1 = x; }
  else if (3.0f <= hh && hh < 4.0f) { r1 = 0; g1 = x; b1 = c; }
  else if (4.0f <= hh && hh < 5.0f) { r1 = x; g1 = 0; b1 = c; }
  else { r1 = c; g1 = 0; b1 = x; }

  float m = v - c;
  float rf = r1 + m;
  float gf = g1 + m;
  float bf = b1 + m;

  outR = (uint16_t)constrain((int)(rf * PWM_MAX + 0.5f), 0, PWM_MAX);
  outG = (uint16_t)constrain((int)(gf * PWM_MAX + 0.5f), 0, PWM_MAX);
  outB = (uint16_t)constrain((int)(bf * PWM_MAX + 0.5f), 0, PWM_MAX);
}