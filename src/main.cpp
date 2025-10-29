#include <Arduino.h>
#include "LEDController.h"
#include "LTTController.h"
#include "WiFiManager.h"
#include "State.h"
#include "PartyController.h"

const int RED_PIN = 5;
const int GREEN_PIN = 6;
const int BLUE_PIN = 7;

const int POT_RED_PIN = 4;
const int POT_GREEN_PIN = 3;
const int POT_BLUE_PIN = 0;

const int MOVING_AVERAGE_SIZE = 8; // Size of the moving average window

int pot1Values[MOVING_AVERAGE_SIZE] = {0};
int pot2Values[MOVING_AVERAGE_SIZE] = {0};
int pot3Values[MOVING_AVERAGE_SIZE] = {0};
int potIndex = 0;

int lastPot1 = 0;
int lastPot2 = 0;
int lastPot3 = 0;

// Party speed mapping (pot -> degrees per second)
const float PARTY_SPEED_MIN = 10.0f;
const float PARTY_SPEED_MAX = 720.0f; // fast full-rotation speeds

LEDController ledController(
    RED_PIN, GREEN_PIN, BLUE_PIN,
    0, 1, 2);

LTTController lttController(ledController);
WiFiManager wifiManager(ledController);
StateHandler stateHandler(ledController);
PartyController partyController(ledController);

int readAveragedADC(int pin, int samples = 4)
{
  int32_t sum = 0;
  for (int i = 0; i < samples; i++)
  {
    sum += analogReadMilliVolts(pin);
  }
  return sum / samples;
}

int calculateMovingAverage(int *values, int size)
{
  int sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += values[i];
  }
  return sum / size;
}

void setup()
{
  Serial.begin(115200);
  ledController.begin();
  stateHandler.begin();
  partyController.begin();

  analogSetAttenuation(ADC_2_5db);
  analogSetPinAttenuation(POT_RED_PIN, ADC_2_5db);
  analogSetPinAttenuation(POT_GREEN_PIN, ADC_2_5db);
  analogSetPinAttenuation(POT_BLUE_PIN, ADC_2_5db);
}

void loop()
{
  static unsigned long lastUpdate = 0;
  const unsigned long UPDATE_INTERVAL = 20;

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL)
  {
    lastUpdate = currentMillis;
    stateHandler.update();

    if (stateHandler.getCurrentMode() == OperationMode::WIFI)
    {
      wifiManager.update();
    }

    int pot1 = readAveragedADC(POT_RED_PIN);
    int pot2 = readAveragedADC(POT_GREEN_PIN);
    int pot3 = readAveragedADC(POT_BLUE_PIN);

    pot1 = map(constrain(pot1, 5, 950), 5, 950, 0, 2047); // Left pot (meant for Red)
    pot2 = map(constrain(pot2, 5, 950), 5, 950, 0, 2047); // Middle pot (meant for Green)
    pot3 = map(constrain(pot3, 5, 950), 5, 950, 0, 2047); // Right pot (meant for Blue)

    // Update moving average arrays
    pot1Values[potIndex] = pot1;
    pot2Values[potIndex] = pot2;
    pot3Values[potIndex] = pot3;
    potIndex = (potIndex + 1) % MOVING_AVERAGE_SIZE;

    // Calculate moving averages
    pot1 = calculateMovingAverage(pot1Values, MOVING_AVERAGE_SIZE);
    pot2 = calculateMovingAverage(pot2Values, MOVING_AVERAGE_SIZE);
    pot3 = calculateMovingAverage(pot3Values, MOVING_AVERAGE_SIZE);

    static bool wasInWiFiMode = false;
    bool isInWiFiMode = stateHandler.getCurrentMode() == OperationMode::WIFI;

    if (isInWiFiMode && !wasInWiFiMode)
    {
      wifiManager.begin();
      ledController.checkAndUpdatePowerLimit();
    }
    else if (!isInWiFiMode && wasInWiFiMode)
    {
      wifiManager.stop();
      ledController.setPWMDirectly(0, 0, 0);
      ledController.checkAndUpdatePowerLimit();
    }
    wasInWiFiMode = isInWiFiMode;

    // PARTY mode entry/exit handling
    static bool wasInPartyMode = false;
    bool isInPartyMode = stateHandler.getCurrentMode() == OperationMode::PARTY;

    if (isInPartyMode && !wasInPartyMode) {
      // Entering PARTY
      partyController.start();
    } else if (!isInPartyMode && wasInPartyMode) {
      // Leaving PARTY
      partyController.stop();
    }
    wasInPartyMode = isInPartyMode;

    // If we're in PARTY, map pots to party parameters:
    // - pot1 -> max brightness (0..1)
    // - pot2 -> speed (mapped to PARTY_SPEED_MIN..PARTY_SPEED_MAX deg/sec)
    // - pot3 -> saturation (0..1)
    if (isInPartyMode) {
      const float PWM_MAX_F = 2047.0f;
      float brightness = constrain(pot1 / PWM_MAX_F, 0.0f, 1.0f);
      float saturation = constrain(pot3 / PWM_MAX_F, 0.0f, 1.0f);
      float speed = (pot2 / PWM_MAX_F) * (PARTY_SPEED_MAX - PARTY_SPEED_MIN) + PARTY_SPEED_MIN;

      partyController.setBrightness(brightness);
      partyController.setSaturation(saturation);
      partyController.setSpeed(speed);
    }

    switch (stateHandler.getCurrentMode())
    {
    case OperationMode::RGB:
      // pot1 is LEFT (physically red), pot3 is RIGHT (physically blue)
      // since the LED pins are now swapped in begin(), we need to swap these too
      ledController.setPWMDirectly(pot3, pot2, pot1); // Swap values to match physical layout
      break;
    case OperationMode::LTT:
      lttController.updateLTT(pot1, pot2, pot3);
      break;
    case OperationMode::PARTY:
      partyController.update();
      break;
    case OperationMode::WIFI:
      // LED control happens via WiFi in WIFI mode
      break;
    }
  }
  delay(2);
}