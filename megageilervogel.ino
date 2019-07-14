#include <FastLED.h>
#include <ADC.h>
#include <Audio.h>
#include <SoftwareSerial.h>
#include <Adafruit_Pixie.h>
#include "Adafruit_Trellis.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <math.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define POTENTIOMETER_PIN 37
#define MIC_PIN 17

// analog - digital
ADC *adc = new ADC();

// motion sensor
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(&Wire2, 1000);
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// button board
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet pad = Adafruit_TrellisSet(&matrix0);
#define PAD_CONNECTED true

// 3W pixels
SoftwareSerial pixieSerial(-1, 5);
SoftwareSerial pixieSerial2(-1, 20);
SoftwareSerial pixieSerial3(-1, 21);
Adafruit_Pixie eyes = Adafruit_Pixie(1, &pixieSerial);
Adafruit_Pixie fiberHead = Adafruit_Pixie(1, &pixieSerial3);
Adafruit_Pixie fiberTail = Adafruit_Pixie(1, &pixieSerial2);

#define NUM_LEDS 600
#define NUM_LEDS_PER_STRIP 120
#define NUM_STRIPS 5
#define MAX_BRIGHTNESS 55
#define TEENSY_LED 13

CRGB leds[NUM_LEDS];

#include "segment.h"

typedef void (*Mode[2])(void);

// leds per strip
uint8_t ledsPerStrip[] = {68, 68, 100, 95};

// buttons
#define EYE_STATE 15

// active or not active
uint8_t buttonState[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// global state
int8_t currentMode = 0;
int8_t previousMode = 0;
uint8_t currentBrightness = MAX_BRIGHTNESS;
uint16_t currentDelay = 0; // delay between frames
uint8_t shouldClear = 1;   // clear all leds between frames
uint8_t shouldShow = 1;
uint8_t usePixies = 0;        // pixies on
uint8_t usePotentiometer = 1; // use potentiometer to get brightness value for next frame
CRGB *overridePixieColor = 0; // override the default pixie red

// Debug stuff to count LEDS properly
static int globalP = 0;

int accelTestMV = 0;

void none() {}

Mode modes[] = {
    // {accelLerp, accelLerpSetup}, // y

    // {wingTest, none},

    // {accelOrientation, accelOrientationSetup},
    // {newAccel, newAccelSetup},
    // {runner, none},
    {fiberPulse, fiberPulseSetup}, // ?
    {fiberBlink, fiberBlinkSetup},

    {heartbeat, heartbeatSetup}, //y
    {sinelon, sinelonSetup},     // y

    {newLerp, newLerpSetup},   // Y
    {lerpTest, lerpTestSetup}, // y

    {pride, prideSetup}, // ?
    // {colorWheelPulsing, colorWheelPulsingSetup},
    {colorWheelNew, colorWheelNewSetup}, //y
    {iceSparks, iceSparksSetup},
    {fire, fireSetup},           // ?
    {accelLerp, accelLerpSetup}, // y
    {betterAudio, betterAudioSetup},
    {newAudio, newAudioSetup},

    // {juggle, juggleSetup},
    // {flash2, flash2Setup},
    // {bpm, bpmSetup},   // ?                                   /
    // {fireWithPotentiometer, fireWithPotentiometerSetup}, // y
    // {fiberMode, fiberModeSetup}, // ?

    // {none, none},
    // {fibertest, none},
    // {wingTest, none},
    // {thunder, thunderSetup},
    // {moveToOuter, moveToOuterSetup},
    // {colorWheelWithSparkels, colorWheelWithSparkelsSetup}, // Y

    // {ftest, none},

    // {wingTest, none},

    // {simpleAudio, simpleAudioSetup}, // y

    // {wingTest, none},
    // {linearTest, none},
    // {pixieTest, pixieTestSetup},
    // {ftest, none},
    // {none, none},
    // {sparks, sparksSetup},
    // {rainbowSparks, rainbowSparksSetup},
    // {randomBluePixels, randomBluePixelsSetup},
    // {none, none},
    // {flashTest, flashTestSetup},
    // {simpleAudio, simpleAudioSetump}, // y

    // {fireNoise, fireNoiseSetup},
};

#include "libs/util.cpp"

// the setup routine runs once when you press reset:
void setup()
{
  while (!Serial && (millis() <= 2000))
    ; // Wait for Serial interface
  Serial.begin(115200);

  Serial.println("boot bird");
  Serial.println("LED init");
  // init all other leds

  // FastLED.addLeds<OCTOWS2811>(leds, NUM_LEDS_PER_STRIP);

  // FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>

  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 5000);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  clear();
  FastLED.clear();
  FastLED.show();

  if (PAD_CONNECTED)
  {
    Serial.println("pad init");
    // init button pads and flash them once
    pad.begin(0x70);
    for (uint8_t i = 0; i < 16; i++)
    {
      pad.setLED(i);
      pad.writeDisplay();
      delay(25);
    }
    for (uint8_t i = 0; i < 16; i++)
    {
      pad.clrLED(i);
      pad.writeDisplay();
      delay(25);
    }
  }
  else
  {
    Serial.println("pad disabled!");
  }
  Serial.println("motion init");
  delay(100);
  // init motion sensor
  if (lsm.begin())
  {
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  }
  else
  {
    Serial.println("LSM SENSOR ERROR!");
  }

  // initialize the digital pin as an output.
  pinMode(TEENSY_LED, OUTPUT);

  Serial.println("3W LED init");

  // init 3w leds serial connection
  pixieSerial.begin(115200);
  pixieSerial2.begin(115200);
  pixieSerial3.begin(115200);

  adc->setAveraging(16, ADC_1);                                    // set number of averages
  adc->setResolution(16, ADC_1);                                   // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_1);     // change the sampling speed
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  adc->adc1->recalibrate();

  AudioMemory(8);

  // adc->setAveraging(16, ADC_1);
  // adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1);
  // adc->setResolution(10, ADC_1); // adc->setResolution(10, ADC_1):

  delay(10); // if we fucked it up - great idea by fastled :D

  // start with mode number 0
  modes[0][1]();
}

void PrintE()
{
  float e, n;
  int b, bands, bins, count = 0, d;

  bands = 30; // Frequency bands; (Adjust to desired value)
  bins = 512; // FFT bins; (Adjust to desired value)

  e = FindE(bands, bins); // Find calculated E value
  if (e)
  {                                  // If a value was returned continue
    Serial.printf("E = %4.4f\n", e); // Print calculated E value
    for (b = 0; b < bands; b++)
    { // Test and print the bins from the calculated E
      n = pow(e, b);
      d = int(n + 0.5);

      Serial.printf("%4d ", count); // Print low bin
      count += d - 1;
      Serial.printf("%4d\n", count); // Print high bin
      ++count;
    }
  }
  else
    Serial.println("Error\n"); // Error, something happened
}

float FindE(int bands, int bins)
{
  float increment = 0.1, eTest, n;
  int b, count, d;

  for (eTest = 1; eTest < bins; eTest += increment)
  { // Find E through brute force calculations
    count = 0;
    for (b = 0; b < bands; b++)
    { // Calculate full log values
      n = pow(eTest, b);
      d = int(n + 0.5);
      count += d;
    }
    if (count > bins)
    {                     // We calculated over our last bin
      eTest -= increment; // Revert back to previous calculation increment
      increment /= 10.0;  // Get a finer detailed calculation & increment a decimal point lower
    }
    else if (count == bins)    // We found the correct E
      return eTest;            // Return calculated E
    if (increment < 0.0000001) // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
      return (eTest - increment);
  }
  return 0; // Return error 0
}

void checkSerial()
{
  if (Serial.available() > 0)
  {
    Serial.read();
    //nextMode(1);
    // Serial.println(currentMode);
    globalP++;
    Serial.print("CURRENT POS");
    Serial.println(globalP);
    nextMode(1);
    // accelTestMV = 1;
  }
}

static uint16_t potentiometer = 0;

void checkPotentiometer()
{
  potentiometer = adc->analogRead(POTENTIOMETER_PIN, ADC_1);

  uint8_t brightness = map(potentiometer, 0, 65535, 0, 255); // potentiometer / 4;
  // Serial.println(brightness);
  currentBrightness = brightness;
  if (usePotentiometer == 1)
  {
    LEDS.setBrightness(brightness);
    // fiberHead.setBrightness(brightness);
    // fiberTail.setBrightness(brightness);
  }
}

void fiberModeSetup()
{
  currentDelay = 25;
}

void fiberMode()
{
  static int8_t hue = 0;
  hue++;
  CRGB c;
  c.setHSV(hue, 240, 255);
  fiberHead.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberHead.setBrightness(currentBrightness);
  fiberTail.setBrightness(currentBrightness);
  // fibertest();
  fiberHead.show();
  fiberTail.show();
  leds[fiberleft(0)] = CHSV(hue * 2, 240, 255);
  leds[fiberleft(1)] = CHSV(hue, 240, 255);
  leds[fiberleft(2)] = CHSV(hue * 3, 200, 255);
  leds[fiberleft(3)] = CHSV(hue, 140, 255);
  leds[fiberright(0)] = CHSV(hue * 2, 240, 255);
  leds[fiberright(1)] = CHSV(hue, 240, 255);
  leds[fiberright(2)] = CHSV(hue * 3, 200, 255);
  leds[fiberright(3)] = CHSV(hue, 140, 255);
}

void checkButtons()
{
  if (pad.readSwitches())
  {
    // if (pad.justPressed(0))
    // {
    //   nextMode(1);
    //   return;
    // }

    //  debug
    // if (pad.justPressed(0))
    // {
    //   globalP = 0;
    // }
    // if (pad.justPressed(1))
    // {
    //   globalP++;
    // }
    // if (pad.justPressed(2))
    // {
    //   globalP--;
    //   if (globalP < 0)
    //   {
    //     globalP = 0;
    //   }
    // }
    // Serial.println(globalP);
    // return;
    // buttons with a direct mode mapping
    for (uint8_t i = 0; i < 15; i++)
    {

      if (pad.justPressed(i))
      {
        // Serial.print("BUTTON ");
        // Serial.println(i);
        pad.setLED(i);
        // if (i == 8)
        // {
        //   nextMode(1);
        // }
        // else if (i == 9)
        // {
        //   nextMode(-1);
        // }
        // else
        if (i == 12)
        {
          // globalP++;
          flash();
        }
        else if (i == 13)
        {
          // globalP = 0;
          strobo();
        }
        else if (i < 12 || i == 14)
        {
          if (i == 14)
            setMode(12);
          else
            setMode(i);
        }
      }

      if (pad.justReleased(i))
      {
        pad.clrLED(i);
      }
    }
  }

  for (uint8_t i = 15; i < 16; i++)
  {
    if (pad.justPressed(i))
    {
      if (pad.isLED(i))
      {
        pad.clrLED(i);
        buttonState[i] = 0;
      }
      else
      {
        pad.setLED(i);
        buttonState[i] = 1;
      }
    }
  }
  pad.writeDisplay();
}

void colorWheel()
{
  static uint8_t hue = 0;
  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    {
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(255, 255, 255);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV((32 * i) + hue + j, 192, 255);
    }
  }
}

void flashTestSetup()
{
  // currentDelay = 2;
  currentDelay = 5;
  shouldClear = false;
}

void flashTest()
{
  // clear();
  fadeToBlackBy(leds, NUM_LEDS, 2);
  EVERY_N_MILLISECONDS(1000)
  {
    for (int j = 0; j < 20; j++)
    {
      int x = random(0, NUM_LEDS);
      leds[x] = CRGB::White;
    }
  }
}
void randomBluePixelsSetup()
{
  // currentDelay = 2;
  currentDelay = 0;
  shouldClear = false;
}

void randomBluePixels()
{
  // clear();
  fadeToBlackBy(leds, NUM_LEDS, 40);
  for (int j = 0; j < 3; j++)
  {
    int x = random(0, NUM_LEDS);
    leds[x] = CRGB::Blue;
  }
}

uint16_t stripoffset(uint8_t n)
{
  return NUM_LEDS_PER_STRIP * n;
}

// looking from behind
// left:
//   strip outside - 0
//     front: 1-45 (first pixel is dead), (running outwards)
//     up-down  46-71 (running outwards)
//     down-up: 72-94 (running inwards)
//     fiber: 95-98
//   strip  inside - 1 (+120)
//     down-up: 0-31 (running outwards)
//     top-down 31-62 (running inwards)
// right:
//   strip outside - 2 (+240)
//     front: 0-44 (running outwards)
//     up-down:  45-71 (going outwards)
//     down-up: 72-99 (running inwards)
//     fiber: 100-103
//   strip inside -  3 (+360)
//     up-down: 0-33 (going outwards)
//     down->up: 34-67 (running inwards)
// body:
//   strip 4 (+480)
//     back 0-35
//     front 36-67

void fill(uint16_t from, uint16_t to, CRGB c)
{
  fill_solid(leds + from, to - from, c);
}

void wingTest()
{

  lwFrontInner.lerpTo(255, CRGB::Green);
  lwFrontOuter.lerpTo(255, CRGB::Red);

  // lwMiddleTop.lerpTo(255, CRGB::Blue);

  // lwBackTop.lerpTo(255, CRGB::Yellow);

  // rwMiddleTop.lerpTo(255, CRGB::Blue);
  // rwMiddleBottom.lerpTo(255, CRGB::Green);

  // rwBackTop.lerpTo(255, CRGB::Yellow);
  // rwBackBottom.lerpTo(255, CRGB::Green);

  // fill(LW_FRONT_START, LW_FRONT_PEAK, CRGB::Green);
  // fill(LW_FRONT_PEAK, LW_FRONT_END, CRGB::Red);

  // fill(LW_MIDDLE_START, LW_MIDDLE_PEAK, CRGB::Yellow);
  // fill(LW_MIDDLE_PEAK, LW_MIDDLE_END, CRGB::Red);

  // fill(LW_BACK_START, LW_BACK_PEAK, CRGB::Red);
  // fill(LW_BACK_PEAK, LW_BACK_END + 20, CRGB::Yellow);

  // fill(RW_FRONT_START, RW_FRONT_PEAK, CRGB::Green);
  // fill(RW_FRONT_PEAK, RW_FRONT_END, CRGB::Red);

  // fill(RW_MIDDLE_START, RW_MIDDLE_PEAK, CRGB::Green);
  // fill(RW_MIDDLE_PEAK, RW_MIDDLE_END, CRGB::Red);

  // fill(RW_BACK_START, RW_BACK_PEAK, CRGB::Red);
  // fill(RW_BACK_PEAK, RW_BACK_END + 20, CRGB::Green);

  // fill(BODY_FRONT_START, BODY_FRONT_END, CRGB::Red);
  // fill(BODY_BACK_START, BODY_BACK_END, CRGB::Green);

  // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  // fill(BODY_START, BODY_END, CRGB::Green);
  // fill(HEAD_START, HEAD_END, CRGB::Red);
}

// #define LW_S1A_START 46
// #define LW_S1A_END 71
// #define LW_S1B_START 72
// #define LW_S1B_END 94
// #define LW_S2A_START 0 + NUM_LEDS_PER_STRIP
// #define LW_S2A_END 31 + NUM_LEDS_PER_STRIP
// #define LW_S2B_START 32 + NUM_LEDS_PER_STRIP
// #define LW_S2B_END 62 + NUM_LEDS_PER_STRIP

// #define RW_FRONT_START (0 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_FRONT_END (44 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1A_START (45 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1A_END (71 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1B_START (72 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S1B_END (99 + (NUM_LEDS_PER_STRIP * 2))
// #define RW_S2A_START (0 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2A_END (33 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2B_START (34 + (NUM_LEDS_PER_STRIP * 3))
// #define RW_S2B_END (67 + (NUM_LEDS_PER_STRIP * 3))

// #define BODY_FRONT_START (36 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_FRONT_END (67 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_BACK_START (0 + NUM_LEDS_PER_STRIP * 4)
// #define BODY_BACK_END (35 + NUM_LEDS_PER_STRIP * 4)

// 5 leds per wing at the same position (0-255)

void leftWingLinear(uint8_t x, CRGB c)
{
  leds[lerp8by8(LW_FRONT_START, LW_FRONT_PEAK, x)] += c;
  leds[lerp8by8(LW_MIDDLE_START, LW_MIDDLE_PEAK, x)] += c;
  leds[lerp8by8(LW_MIDDLE_END, LW_MIDDLE_PEAK, x)] += c;

  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    leds[lerp8by8(LW_BACK_START, LW_BACK_PEAK, xx)] += c;
    leds[lerp8by8(LW_BACK_END, LW_BACK_PEAK, xx)] += c;
  }
}

void rightWingLinear(uint8_t x, CRGB c)
{
  leds[lerp16by8(RW_FRONT_START, RW_FRONT_PEAK, x)] += c;
  leds[lerp16by8(RW_MIDDLE_START, RW_MIDDLE_PEAK, x)] += c;
  leds[lerp16by8(RW_MIDDLE_END, RW_MIDDLE_PEAK, x)] += c;

  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    leds[lerp16by8(RW_BACK_START, RW_BACK_PEAK, xx)] += c;
    leds[lerp16by8(RW_BACK_END, RW_BACK_PEAK, xx)] += c;
  }
}

void linearTest()
{
  leftWingLinear(globalP * 4, CRGB::Red);
  rightWingLinear(globalP * 4, CRGB::Green);
}

// void bodyFront(uint8_t x, CRGB c)
// {
//   leds[lerp16by8(BODY_FRONT_START, BODY_FRONT_END, x)] += c;
// }
// void bodyBack(uint8_t x, CRGB c)
// {
//   leds[lerp16by8(BODY_BACK_END, BODY_BACK_START, x)] += c;
// }

void accelOrientationSetup()
{
  currentDelay = 3;
  usePixies = false;
  shouldClear = false;
}

#define LERP_ACCEL_START 60

void accelOrientation()
{

  fadeToBlackBy(leds, NUM_LEDS, 7);

  sensors_vec_t orientation;
  if (ahrs.getOrientation(&orientation))
  {
    Serial.print("Orientation: ");
    Serial.print(orientation.roll);
    Serial.print(" ");
    Serial.print(orientation.pitch);
    Serial.print(" ");
    Serial.print(orientation.heading);
    Serial.println("");

    float p0 = orientation.pitch - 90.0;

    Serial.print("p0: ");
    Serial.print(p0);
    Serial.println();

    if (p0 > 0)
    {
      bodyFront.lerpTo(p0 * 2, CRGB::Blue);
    }
    else
    {
      bodyBack.lerpToReverse(p0 * -2, CRGB::Blue);
    }
  }

  // pitch is 89 when strait

  // int p = LERP_ACCEL_START + mv * 15.0;

  // Serial.print(mv);
  // Serial.print(" ");
  // Serial.println(p);

  // wingLinearLerpTo(p, CHSV(HUE_BLUE + p, 230, p));

  // if (p > 220)
  // {
  //   lwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  //   rwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  // }

  // for (int i = 0; i < f2; i += 5)
  // {
  //   wingLinearLerp(i, CHSV(hue2 + i / 2, 255, 255));
  //   // leftWingLinear(i, CHSV(hue2 + 100, 255, 255));
  //   // rightWingLinear(i, CHSV(hue2 + 100, 255, 255));
  //   // bodyFront(i, CHSV(hue2 + 100, 255, 255));
  //   // bodyBack(i, CHSV(hue2 + 100, 255, 255));
  // }
}

void newAccelSetup()
{
  currentDelay = 2;
  usePixies = false;
  shouldClear = false;
}

#define LERP_ACCEL_START 60

void newAccel()
{

  fadeToBlackBy(leds, NUM_LEDS, 7);
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float mv = accel.acceleration.x + (9.81 + 0.9);

  Serial.print(accel.gyro.x, 4);
  Serial.print(" ");
  Serial.print(accel.gyro.y, 4);
  Serial.print(" ");
  Serial.print(accel.gyro.z, 4);
  Serial.println("");
  // Serial.print(accel.acceleration.x, 4);
  // Serial.print(" ");
  // Serial.print(accel.acceleration.y, 4);
  // Serial.print(" ");
  // Serial.print(accel.acceleration.z, 4);
  // Serial.println("");
  // static uint8_t hue1 = 0;
  // hue1++;
  // uint8_t hue2 = hue1;

  // uint8_t f2 = uint8_t(LERP_ACCEL_START + (mv * 10.0));

  int p = LERP_ACCEL_START + mv * 15.0;

  // Serial.print(mv);
  // Serial.print(" ");
  // Serial.println(p);

  wingLinearLerpTo(p, CHSV(HUE_BLUE + p, 230, p));

  // if (p > 220)
  // {
  //   lwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  //   rwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  // }

  // for (int i = 0; i < f2; i += 5)
  // {
  //   wingLinearLerp(i, CHSV(hue2 + i / 2, 255, 255));
  //   // leftWingLinear(i, CHSV(hue2 + 100, 255, 255));
  //   // rightWingLinear(i, CHSV(hue2 + 100, 255, 255));
  //   // bodyFront(i, CHSV(hue2 + 100, 255, 255));
  //   // bodyBack(i, CHSV(hue2 + 100, 255, 255));
  // }

  if (mv > 10)
  {
    strobo();
  }
}

enum StroboState
{
  NextOn,
  On,
  NextOff,
  Off
};
class ConcurrentStrobo
{
public:
  elapsedMillis timer;
  int c;
  StroboState state;
  ConcurrentStrobo()
  {
    c = 0;
    timer = 0;
    state = Off;
  }
  void trigger()
  {
    // only if not running anyway
    if (c > 0)
      return;
    c = 5;
    timer = 0;
    state = NextOn;
    usePixies = 1;
  }
  void advance()
  {
    switch (state)
    {
    case On:
      if (timer > 50)
      {
        timer = 0;
        state = NextOff;
      }
      break;
    case Off:
      if (timer > 100 && c > 0)
      {
        timer = 0;
        state = NextOn;
      }
      break;
    case NextOn:
      eyes.setBrightness(255);
      eyes.setPixelColor(0, 255, 255, 255);
      eyes.show();
      state = On;
      timer = 0;
      c--;
      break;
    case NextOff:
      eyes.setBrightness(0);
      eyes.show();
      state = Off;
      timer = 0;
      if (c == 0)
        usePixies = 0;
      break;
    }
  }
};

void accelLerpSetup()
{
  currentDelay = 3;
  // usePixies = false;
  usePixies = 1;
  shouldClear = false;
}
#define LERP_ACCEL_START 50

void accelLerp()
{

  static ConcurrentStrobo s;

  fadeToBlackBy(leds, NUM_LEDS, 35);
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float mv = accel.acceleration.x + (9.81 + 0.9);

  if (accelTestMV == 1)
  {
    accelTestMV = 0;
    mv = 15;
  }

  // Serial.println(accel.acceleration.x);
  static uint8_t hue1 = 0;
  hue1++;
  uint8_t hue2 = hue1;

  uint8_t f2 = uint8_t(LERP_ACCEL_START + (mv * 12.0));
  f2 = constrain(f2, 0, 245);

  for (int i = 0; i < f2; i += 5)
  {
    CRGB c = CHSV(hue2 + i / 2, 255, 255);
    wingLinearLerp(i, c);
    bodyFront.lerpAt(i, c);
    bodyBack.lerpAt(255 - i, c);
    // leftWingLinear(i, CHSV(hue2 + 100, 255, 255));
    // rightWingLinear(i, CHSV(hue2 + 100, 255, 255));
    // bodyFront(i, CHSV(hue2 + 100, 255, 255));
    // bodyBack(i, CHSV(hue2 + 100, 255, 255));
  }

  if (mv > 10)
  {
    s.trigger();
    headLeft.fill(CHSV(240, 240, 255));
    headRight.fill(CHSV(240, 240, 255));
    lwFrontOuter.fill(CHSV(240, 240, 255));
    rwFrontOuter.fill(CHSV(240, 240, 255));
    // strobo();
  }

  s.advance();
}

void lerpTestSetup()
{
  currentDelay = 10;
  shouldClear = false;
}
void lerpTest()
{
  static uint8_t hue1 = 0;
  hue1++;
  uint8_t hue2 = hue1;

  fadeToBlackBy(leds, NUM_LEDS, 25);

  // for (int i = 0; i < 2; i++)
  // {

  // uint8_t f = ease8InOutCubic(p + i);

  uint8_t f = beatsin8(100, 0, 255);
  leftWingLinear(f, CHSV(hue1, 255, 255));
  rightWingLinear(f, CHSV(hue1, 255, 255));

  bodyFront.lerpAt(f, CHSV(hue1, 255, 255));
  bodyBack.lerpAt(255 - f, CHSV(hue1, 255, 255));

  uint8_t f2 = beatsin8(50, 20, 220);
  leftWingLinear(f2, CHSV(hue2 + 100, 255, 255));
  rightWingLinear(f2, CHSV(hue2 + 100, 255, 255));
  // bodyFront(f2, CHSV(hue2 + 100, 255, 255));
  // bodyBack(f2, CHSV(hue2 + 100, 255, 255));

  // }
}

void ftest()
{
  clear();
  CRGB stripColor[5] = {CRGB::Green, CRGB::Red, CRGB::Blue, CRGB::Yellow, CRGB::White};
  for (int i = 0; i < 5; i++)
  {
    leds[globalP + stripoffset(i)] = stripColor[i];

    // CRGB c;
    // // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // // for (int j = 64 + 26 + 5; j < 64 + 4 + 26 + 5; j++)
    // for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    // // for (int j = 0; j < globalP; j++)
    // {
    //   if (i % 4 == 0)
    //   {
    //     c = CRGB::Green;
    //   }
    //   if (i % 4 == 1)
    //   {
    //     c = CRGB::Blue;
    //   }
    //   if (i % 4 == 2)
    //   {
    //     c = CRGB::Red;
    //   }
    //   if (i % 4 == 3)
    //   {
    //     c = CRGB::Yellow;
    //   }

    //   leds[j + stripoffset(i)] = c;
    // }
  }
}

uint16_t xy60x6(uint8_t x, uint8_t y)
{
  if (y > 29)
  {
    x = x + 6;
    y = 59 - y;
  }
  return (x * 30) + y;
}

uint16_t xy(uint8_t x, uint8_t y)
{
  if (x % 2 != 0)
    return (x % 12) * 30 + (y % 30);
  else
    return (x % 12) * 30 + (29 - (y % 30));
}

// looking from front

uint16_t fiberright(int8_t n)
{
  // maybe +2
  return NUM_LEDS_PER_STRIP * 2 + 96 + n;
}

uint16_t fiberleft(uint8_t n)
{
  return 101 + n;
}

void fibertest()
{

  // leds[fiberright(-5)] = CRGB::Green;
  // leds[fiberright(-3)] = CRGB::Green;
  // leds[fiberright(-3)] = CRGB::Green;
  // leds[fiberright(-2)] = CRGB::Green;
  // leds[fiberright(-1)] = CRGB::Green;
  leds[fiberright(0)] = CRGB::Green;
  leds[fiberright(1)] = CRGB::Blue;
  leds[fiberright(2)] = CRGB::Red;
  leds[fiberright(3)] = CRGB::Yellow;

  leds[fiberleft(0)] = CRGB::Green;
  leds[fiberleft(1)] = CRGB::Blue;
  leds[fiberleft(2)] = CRGB::Red;
  leds[fiberleft(3)] = CRGB::Yellow;

  // leds[fiberright(4)] = CRGB::Yellow;
  // leds[fiberright(5)] = CRGB::Yellow;
  // leds[fiberright(6)] = CRGB::Yellow;
  // leds[fiberright(7)] = CRGB::Yellow;

  // leds[fiberleft(0)] = CRGB::Green;
  // leds[fiberleft(1)] = CRGB::Blue;
  // leds[fiberleft(2)] = CRGB::Red;
  // leds[fiberleft(3)] = CRGB::Yellow;

  // leds[fiberright(0)] = CRGB::Green;
  // leds[fiberright(1)] = CRGB::Blue;
  // leds[fiberright(2)] = CRGB::Red;
  // leds[fiberright(3)] = CRGB::Yellow;
}

void runner()
{
  static int pos = 0;
  static uint8_t hue = 0;

  hue++;
  for (int i = 0; i < NUM_STRIPS; i++)
  {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++)
    {
      // char r, g, b;
      // if (j % 3 == 0)
      // {
      //   r = 255;
      // }
      // else if (j % 3 == 1)
      // {
      //   g = 255;
      // }
      // else if (j % 3 == 2)
      // {
      //   b = 255;
      // }

      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0);
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
      //leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(255, 255, 255);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(r, g, b);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 255, 0) ;
    }
    // leds[(i * NUM_LEDS_PER_STRIP) + pos] = CRGB(255, 0, 0);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 1) % NUM_LEDS_PER_STRIP)] = CRGB(0, 255, 0);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 2) % NUM_LEDS_PER_STRIP)] = CRGB(0, 0, 255);
    // leds[(i * NUM_LEDS_PER_STRIP) + ((pos + 3) % NUM_LEDS_PER_STRIP)] = CRGB(0, 255, 0);
  }
  // pos++;
}

uint16_t fps = 0;

void showFps()
{
  Serial.println(fps);
  fps = 0;
}

void prideSetup()
{
  currentDelay = 50;
}

void pride()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  static uint8_t start = 0;
  start++;
  // EVERY_N_MILLISECONDS(60) { start++; }
  start = start % 2;

  uint8_t sat8 = beatsin88(87, 220, 250);
  uint8_t brightdepth = beatsin88(341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88(203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16; //gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis;
  sLastMillis = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88(400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for (uint16_t i = start; i < NUM_LEDS; i += 2)
  {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16 += brightnessthetainc16;
    uint16_t b16 = sin16(brightnesstheta16) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV(hue8, sat8, qadd8(bri8, 100));

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;

    nblend(leds[pixelnumber], newcolor, 64);
  }
}

void colorWheelPulsingSetup()
{
  currentDelay = 40;
}
void colorWheelPulsing()
{
  static uint8_t hue = 0;
  uint8_t pulse = beatsin8(60, 40, 255);
  hue++;

  hue++;
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(5 * i + hue, 240, pulse);
  }
}

void thunderSetup()
{
  currentDelay = 20;
  shouldClear = false;
}

void thunder()
{
  fadeToBlackBy(leds, NUM_LEDS, 10);
  EVERY_N_MILLISECONDS(3000)
  {
    fill_solid(leds, NUM_LEDS, CHSV(255, 10, 100));
    FastLED.show();
    delay(25);
    clear();

    for (int i = 0; i < random8(100); i++)
    {

      leds[random16(NUM_LEDS)] = CHSV(HUE_BLUE + random8(20), 50 + random(50), 155 + random(100));
    }
  }
}

void newLerpSetup()
{
  currentDelay = 10;
  shouldClear = false;
}

class Wave
{
public:
  float speed;
  CHSV color;
  float pos;
  Wave()
  {
    randomize();
  }
  void randomize()
  {
    pos = 0;
    speed = random(10, 100) / 10.0;
    color = blend(CHSV(HUE_RED, 240, 100), CHSV(HUE_BLUE, 240, 100), random8());
  }
  void advance()
  {
    pos += speed;
    if (pos >= 255)
    {
      randomize();
    }
  }
  void draw()
  {
    color.v = 100 + map(pos, 0, 255, 0, 150);
    // rightWingLinearLerp(pos, color);
    bodyBack.lerpAt(pos, color);
    bodyFront.lerpAt(pos, color);
    wingLinearLerp(pos, color);
  }
};
#define WAVES 5
void newLerp()
{
  fadeToBlackBy(leds, NUM_LEDS, 25);
  static Wave waves[WAVES];
  for (int i = 0; i < WAVES; i++)
  {
    waves[i].advance();
    waves[i].draw();
  }

  fiberLeft.fill(waves[0].color);
  fiberRight.fill(waves[0].color);

  // uint8_t x = 0;
  // x++;
  // uint8_t v = map(beatsin8(30), 0, 255, 130, 30);
  // CHSV c = CHSV(212, 240, 170);
  // CRGB cc = c;
  // cc %= 230;

  // bodyFront.fill(cc);
  // bodyBack.fill(cc);

  headLeft.fill(CRGB::BlueViolet);
  headRight.fill(CRGB::DarkRed);

  // } int b = beat8(120);
  // leftWingLinearLerp(b, CRGB::Red);
  // int b2 = beat8(140);
  // leftWingLinearLerp(b2, CRGB::Green);
}

void moveToOuterSetup()
{
  currentDelay = 3;
  shouldClear = false;
}

void moveToOuter()
{
  static int x = 0;
  static int maxX = 255;
  x += 3;
  if (x >= maxX)
  {
    x = 0;
    maxX -= 5;

    if (maxX < 0)
    {
      maxX = 255;
      clear();
    }
  }

  int idx = lwFrontInner.lerp(x);
  fadeToBlackBy(leds + lwFrontInner.start, map(maxX, 0, 255, 0, lwFrontInner.size), 15);
  leds[idx] = CRGB::Red;

  // Serial.print("x=");
  // Serial.print(x);
  // Serial.print(" maxx=");
  // Serial.print(maxX);
  // Serial.print(" idx=");
  // Serial.print(idx);

  // Serial.print(" start=");
  // Serial.print(lwFrontInner.start);

  // Serial.print(" mapped=");
  // Serial.println(map(maxX, 0, 255, 0, lwFrontInner.size));
}

void colorWheelWithSparkelsSetup()
{
  currentDelay = 20;
}
void colorWheelWithSparkels()
{
  static uint8_t hue = 0;
  hue++;

  hue++;
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(5 * i + hue, 240, 150);
  }
  for (int j = 0; j < 3; j++)
  {
    int x = random(0, NUM_LEDS);
    leds[x] = CRGB::White;
  }
}
void colorWheelNewSetup()
{
  currentDelay = 20;
}
void colorWheelNew()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(5 * i + beat8(17), 240, 150);
  }

  CRGB c = leds[bodyBack.end - 1];
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setBrightness(255);
  fiberTail.show();
}

void iceSparksSetup()
{
  static CRGB c = CRGB(10, 10, 240);
  usePotentiometer = 0;
  FastLED.setBrightness(255);
  currentDelay = 50;
  shouldClear = false;
  overridePixieColor = &c;
}

void iceSparks()
{
  fadeToBlackBy(leds, NUM_LEDS, 128);

  static uint8_t sat = 0;
  sat++;
  for (int j = 0; j < 35; j++)
  {
    leds[random(0, NUM_LEDS)] = CHSV(170 + (random(20) - 10), random(255), 250);
  }
}

void basicPowerLedMode()
{
  CRGB c(255, 0, 0);

  if (overridePixieColor)
    c = *overridePixieColor;

  if (buttonState[EYE_STATE] == 1)
  {
    eyes.setBrightness(beatsin8(20, 50, 120));
  }
  else
  {
    eyes.setBrightness(0);
  }

  eyes.setPixelColor(0, c.r, c.g, c.b);
  eyes.show();
}

void sparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 10;
  usePotentiometer = 0;
}

void sparks()
{
  for (int j = 0; j < 30; j++)
  {
    leds[random(0, NUM_LEDS)] = CRGB::White;
  }
}

void fiberBlinkSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 5;
  usePotentiometer = 0;
  shouldClear = true;
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
}
void fiberBlink()
{
  int fiber = random(0, 4 + 4 + 2);
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
  if (fiber < 4)
  {
    leds[fiberleft(fiber)] = CRGB(255, 255, 255);
    // leds[fiberleft(fiber)] = CRGB(0, 0, 255);
  }
  else if (fiber >= 4 && fiber < 8)
  {
    // leds[fiberright(fiber - 4)] = CRGB(255, 0, 0);
    leds[fiberright(fiber - 4)] = CRGB(255, 255, 255);
  }
  else if (fiber == 8)
  {
    fiberTail.setPixelColor(0, 255, 255, 255);
    fiberTail.setBrightness(200);
  }
  else if (fiber == 9)
  {
    fiberHead.setPixelColor(0, 255, 255, 255);
    fiberHead.setBrightness(100);
  }

  fiberHead.show();
  fiberTail.show();
}

void pixieTestSetup()
{
  usePixies = 1;
  currentDelay = 10;
  shouldClear = true;
}
void pixieTest()
{

  eyes.setPixelColor(0, 255, 0, 0);
  fiberTail.setPixelColor(0, 0, 255, 0);
  fiberHead.setPixelColor(0, 0, 0, 255);
  eyes.setBrightness(50);
  fiberTail.setBrightness(0);
  fiberHead.setBrightness(0);
  // EVERY_N_MILLISECONDS(500)
  // {

  // all three must be shown in this order - otherwise it will flicker
  fiberTail.show();
  fiberHead.show();
  eyes.show();
  // }
}

void fiberPulseSetup()
{
  usePixies = 1;
  FastLED.setBrightness(255);
  currentDelay = 10;
  usePotentiometer = 0;
  shouldClear = true;
  fiberHead.setBrightness(0);
  fiberTail.setBrightness(0);
}
void fiberPulse()
{
  static uint8_t hue = 0;
  static uint8_t pulse = 255;
  static int8_t dir = -1;
  static int dirs[10] = {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1};
  static byte pulses[10] = {random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8(), random8()};

  hue++;
  for (int i = 0; i < 10; i++)
  {
    pulses[i] += dirs[i];

    if (pulses[i] < 10)
    {
      dirs[i] = 1;
    }
    if (pulses[i] > 253)
    {
      dirs[i] = -1;
    }

    CRGB c = CHSV(16 * i + hue, 240, pulses[i]);

    if (i < 4)
    {
      leds[fiberleft(i)] = c;
      // leds[fiberleft(fiber)] = CRGB(0, 0, 255);
    }
    else if (i >= 4 && i < 8)
    {
      // leds[fiberright(fiber - 4)] = CRGB(255, 0, 0);
      leds[fiberright(i - 4)] = c;
    }
    else if (i == 8)
    {
      fiberTail.setPixelColor(0, c.r, c.g, c.b);
      fiberTail.setBrightness(255);
    }
    else if (i == 9)
    {
      fiberHead.setPixelColor(0, c.r, c.g, c.b);
      fiberHead.setBrightness(255);
    }
  }
  fiberHead.show();
  fiberTail.show();
}

void rainbowSparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 15;
  usePotentiometer = 0;
}
void rainbowSparks()
{
  static uint8_t hue = 0;
  hue++;
  for (int j = 0; j < 40; j++)
  {
    leds[random(0, NUM_LEDS)] = CHSV(hue, 210, 255);
  }
}

void bpmSetup()
{
  currentDelay = 10;
}

void bpm()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  // float mv = accel.acceleration.x + accel.acceleration.y + accel.acceleration.z + 9.81;
  float mv = accel.acceleration.x + (9.81 + 0.9);
  // Serial.println(mv);

  if (mv > 10)
  {
    strobo();
  }

  static uint8_t gHue = 0;
  uint8_t BeatsPerMinute = 100;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 32, 255);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette(palette, gHue + (i * 4), beat - gHue + (i * 10));
  }
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

void juggleSetup()
{
  currentDelay = 50;
}

void juggle()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++)
  {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void pixiesOff()
{
  eyes.setBrightness(0);
  eyes.show();
  fiberTail.setBrightness(0);
  fiberTail.show();
  fiberHead.setBrightness(0);
  fiberHead.show();
}

void sinelonSetup()
{
  shouldClear = false;
  FastLED.setBrightness(220);
  usePotentiometer = false;

  pixiesOff();
}

void sinelon()
{
  static uint8_t gHue = 0;
  fadeToBlackBy(leds, NUM_LEDS, 20);

  // int pos = beatsin16(16, 0, NUM_LEDS);
  int pos = beatsin16(map(potentiometer, 0, 65535, 2, 60), 0, NUM_LEDS);
  leds[pos] += CHSV(gHue, 255, 192);
  EVERY_N_MILLISECONDS(3) { gHue++; }
}

void flash()
{
  eyes.setBrightness(255);
  eyes.setPixelColor(0, 255, 255, 255);
  eyes.show();
  delay(100);
  eyes.setBrightness(0);
  eyes.show();
}

CRGB stroboC[5] = {CRGB(0, 0, 255), CRGB(255, 0, 0), CRGB(0, 255, 255), CRGB(255, 0, 255), CRGB(255, 255, 255)};

void strobo()
{

  for (int i = 0; i < 5; i++)
  {
    delay(50);
    eyes.setBrightness(255);
    eyes.setPixelColor(0, 255, 255, 255);
    // eyes.setPixelColor(0, 126, 6, 229);
    // eyes.setPixelColor(0, stroboC[i].r, stroboC[i].g, stroboC[i].b);
    eyes.show();
    delay(50);
    eyes.setBrightness(0);
    eyes.show();
  }
}

CRGBPalette16 firePal;

void fireSetup()
{
  currentDelay = 40;
  usePixies = 1;
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::Grey);
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::White);
  // firePal = CRGBPalette16(CRGB::Black, CRGB::Red, CRGB::Yellow, CRGB::White);

  // firePal = CRGBPalette16(CRGB::Black,
  //                         CRGB::Black,
  //                         CRGB::Black,
  //                         CRGB::DarkBlue,
  //                         CRGB::MidnightBlue,
  //                         CRGB::Black,
  //                         CRGB::MediumBlue,
  //                         CRGB::Teal,
  //                         CRGB::CadetBlue,
  //                         CRGB::Blue,
  //                         CRGB::DarkCyan,
  //                         CRGB::CornflowerBlue,
  //                         CRGB::Aquamarine,
  //                         CRGB::SeaGreen,
  //                         CRGB::Aqua,
  //                         CRGB::LightSkyBlue);
  // firePal = LavaColors_p;
  firePal = HeatColors_p;
  // firePal = ForestColors_p;
  // firePal = OceanColors_p;
}

#define COOLING 4
#define SPARKING 80

void fire()
{
  static byte heat[NUM_LEDS];
  random16_add_entropy(random());
  // cool down
  for (int x = 0; x < NUM_LEDS; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, COOLING));
  }
  for (int x = 0; x < NUM_LEDS; x++)
  {
    if (x == 0)
    {
      heat[0] = (heat[0] + heat[1]) / 2;
    }
    if (x == NUM_LEDS - 1)
    {
      heat[NUM_LEDS - 1] = (heat[NUM_LEDS - 1] + heat[NUM_LEDS - 2]) / 2;
    }
    if (x > 0 && x < NUM_LEDS - 1)
    {
      heat[x] = (heat[x - 1] + heat[x] + heat[x + 1]) / 3;
    }
  }

  // // drift up and difuse
  // for (int x = 0; x < X; x++)
  // {
  //   for (int k = Y - 1; k >= 2; k--)
  //   {
  //     int kk = k;
  //     int dir = 1;
  //     if (x % 2 != 0)
  //     {
  //       kk = 29 - k;
  //       dir = -1;
  //     }
  //     heat[(kk + x * Y)] = (heat[(kk + x * Y) - dir] + heat[(kk + x * Y) - (2 * dir)] + heat[(kk + x * Y) - (2 * dir)]) / 3;
  //   }
  // }

  // ignite wings
  for (int i = 0; i < 5; i++)
  {
    if (random8() < SPARKING)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }
  if (random8() < SPARKING)
  {
    int x = random16(NUM_LEDS_PER_STRIP * 4);
    heat[x] = qadd8(heat[x], random8(80, 190));
  }

  // corpus
  for (int i = 0; i < 6; i++)
  {
    if (random8() < SPARKING)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4, NUM_LEDS_PER_STRIP * 5);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }

  // tail
  byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  CRGB c = ColorFromPalette(firePal, colorindex);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setBrightness(255);
  fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  eyes.setBrightness(100);
  eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  eyes.show();

  // map to pixels
  for (int j = 0; j < NUM_LEDS; j++)
  {
    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(firePal, colorindex);
    leds[j] = color;
  }
}

void fireWithPotentiometerSetup()
{
  fireSetup();
  usePotentiometer = 0;
}

void fireWithPotentiometer()
{

  int sparking = potentiometer / 4;
  int bodyCooling = 5;
  int wingCooling = 7;

  static byte heat[NUM_LEDS];
  random16_add_entropy(random());
  // cool down
  for (int x = 0; x < NUM_LEDS; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, wingCooling));
  }
  for (int x = NUM_LEDS * 4; x < NUM_LEDS_PER_STRIP * 5; x++)
  {
    heat[x] = qsub8(heat[x], random8(0, bodyCooling));
  }
  for (int x = 0; x < NUM_LEDS; x++)
  {
    if (x == 0)
    {
      heat[0] = (heat[0] + heat[1]) / 2;
    }
    if (x == NUM_LEDS - 1)
    {
      heat[NUM_LEDS - 1] = (heat[NUM_LEDS - 1] + heat[NUM_LEDS - 2]) / 2;
    }
    if (x > 0 && x < NUM_LEDS - 1)
    {
      heat[x] = (heat[x - 1] + heat[x] + heat[x + 1]) / 3;
    }
    // if (x > 1 && x < NUM_LEDS - 2)
    // {
    //   heat[x] = (heat[x - 2] + heat[x - 1] + heat[x] + heat[x + 1] + heat[x + 2]) / 5;
    // }
  }

  // // drift up and difuse
  // for (int x = 0; x < X; x++)
  // {
  //   for (int k = Y - 1; k >= 2; k--)
  //   {
  //     int kk = k;
  //     int dir = 1;
  //     if (x % 2 != 0)
  //     {
  //       kk = 29 - k;
  //       dir = -1;
  //     }
  //     heat[(kk + x * Y)] = (heat[(kk + x * Y) - dir] + heat[(kk + x * Y) - (2 * dir)] + heat[(kk + x * Y) - (2 * dir)]) / 3;
  //   }
  // }

  // ignite wings
  for (int i = 0; i < 7; i++)
  {
    if (random8() < sparking)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4);
      heat[x] = qadd8(heat[x], random8(50, 190));
    }
  }

  // corpus
  for (int i = 0; i < 6; i++)
  {
    if (random8() < sparking)
    {
      int x = random16(NUM_LEDS_PER_STRIP * 4, NUM_LEDS_PER_STRIP * 5);
      heat[x] = qadd8(heat[x], random8(50, 150));
    }
  }

  // tail
  byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  CRGB c = ColorFromPalette(firePal, colorindex);
  fiberTail.setPixelColor(0, c.r, c.g, c.b);
  fiberTail.setBrightness(255);
  fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  eyes.setBrightness(100);
  eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  eyes.show();

  // map to pixels
  for (int j = 0; j < NUM_LEDS; j++)
  {
    byte colorindex = scale8(heat[j], 240);
    CRGB color = ColorFromPalette(firePal, colorindex);
    leds[j] = color;
  }
}

uint8_t frequency = 20;
uint8_t flashes = 8;
unsigned int dimmer = 1;

uint16_t ledstart;
uint8_t ledlen;

void flash2Setup()
{
  currentDelay = 0;
}
void flash2()
{

  ledstart = random16(NUM_LEDS);
  ledlen = random16(NUM_LEDS - ledstart);

  for (int flashCounter = 0; flashCounter < random8(3, flashes); flashCounter++)
  {
    if (flashCounter == 0)
      dimmer = 5;
    else
      dimmer = random8(1, 3);

    fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 255 / dimmer));
    FastLED.show();
    delay(random8(4, 10));
    fill_solid(leds + ledstart, ledlen, CHSV(255, 0, 0));
    FastLED.show();

    if (flashCounter == 0)
      delay(150);

    delay(50 + random8(100));
  }

  delay(random8(frequency) * 100);
}

void reportSensor()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  // print out accelleration data
  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" ");
  Serial.print("  \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" ");
  Serial.print("  \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println("  \tm/s^2");

  // print out gyroscopic data
  Serial.print("Gyro  X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" ");
  Serial.print("  \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" ");
  Serial.print("  \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" *C");

  Serial.println("**********************\n");
}

void fireNoiseSetup()
{
  currentDelay = 20;
  shouldClear = false;
}

CRGBPalette16 fireNoisePal = CRGBPalette16(
    CRGB::Black, CRGB::Black, CRGB::Black, CHSV(0, 255, 4),
    CHSV(0, 255, 8), CRGB::Red, CRGB::Red, CRGB::Red,
    CRGB::DarkOrange, CRGB::Orange, CRGB::Orange, CRGB::Orange,
    CRGB::Yellow, CRGB::Yellow, CRGB::Gray, CRGB::Gray);

uint32_t xscale = 20; // How far apart they are
uint32_t yscale = 3;  // How fast they move

void fireNoise()
{

  uint8_t index = 0;

  for (int i = 0; i < NUM_LEDS; i++)
  {
    index = inoise8(i * xscale, millis() * yscale * NUM_LEDS / 255);                                       // X location is constant, but we move along the Y at the rate of millis()
    leds[i] = ColorFromPalette(fireNoisePal, min(i * (index) >> 6, 255), i * 255 / NUM_LEDS, LINEARBLEND); // With that value, look up the 8 bit colour palette value and assign it to the current LED.
  }

  // // tail
  // byte colorindex = scale8(heat[NUM_LEDS_PER_STRIP], 240);
  // CRGB c = ColorFromPalette(firePal, colorindex);
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setBrightness(255);
  // fiberTail.show();

  // // eyes
  // // if (buttonState[EYE_STATE] == 1)
  // // {
  // byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  // CRGB cc = ColorFromPalette(firePal, ci);

  // // eyes.setBrightness(currentBrightness / 3);
  // eyes.setBrightness(100);
  // eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // // }
  // // else
  // // {
  // //   eyes.setBrightness(0);
  // // }

  // eyes.show();

  // // map to pixels
  // for (int j = 0; j < NUM_LEDS; j++)
  // {
  //   byte colorindex = scale8(heat[j], 240);
  //   CRGB color = ColorFromPalette(firePal, colorindex);
  //   leds[j] = color;
  // }

  // void 	fill_noise8 (CRGB *leds, int num_leds, uint8_t octaves, uint16_t x, int scale, uint8_t hue_octaves, uint16_t hue_x, int hue_scale, uint16_t time)
}

// void betterAudioSetup()
// {
//   currentDelay = 10;
//   shouldClear = false;
// }

// elapsedMillis audioFps = 0;

// void betterAudio()
// {
//   static uint8_t hue = 0;
//   fadeToBlackBy(leds, NUM_LEDS, 25);
//   hue++;
//   if (audioFps > 25)
//   {
//     if (audioPeak.available())
//     {
//       audioFps = 0;
//       int monoPeak = audioPeak.read();
//       Serial.println(monoPeak);
//       for (int cnt = 0; cnt < monoPeak; cnt++)
//       {
//         leftWingLinear(cnt, CHSV(hue, 255, 255));
//       }
//     }
//   }
// }

// ---------------------------------
// Functional test
#include "effects/tests.cpp"

AudioInputAnalog adc1(MIC_PIN);
AudioAnalyzePeak audioPeak;
AudioConnection patchCord1(adc1, audioPeak);

// AudioAnalyzeRMS rms;
// AudioConnection patchCord3(adc1, rms);
// AudioAnalyzeFFT256 fft;
// AudioConnection patchCord2(adc1, fft);

#define AUDIO_SAMPLES 60 // samples for the mic buffer
#define MIN_DIST_AUDIO_LEVELS 0.1
#define MIN_AUDIO_LEVEL 0.01

float audioSamples[AUDIO_SAMPLES];
float minAudioAvg = 0;
float maxAudioAvg = 5.0;
byte audioSampleIdx = 0;

void audioUpdate(float sample)
{
  float min, max;

  if (sample < MIN_AUDIO_LEVEL)
    sample = 0;

  audioSamples[audioSampleIdx] = sample;
  if (++audioSampleIdx >= AUDIO_SAMPLES)
    audioSampleIdx = 0;

  min = max = audioSamples[0];
  for (uint8_t i = 1; i < AUDIO_SAMPLES; i++)
  {
    if (audioSamples[i] < min)
      min = audioSamples[i];
    else if (audioSamples[i] > max)
      max = audioSamples[i];
  }
  if ((max - min) < MIN_DIST_AUDIO_LEVELS)
    max = min + MIN_DIST_AUDIO_LEVELS;
  minAudioAvg = (minAudioAvg * (AUDIO_SAMPLES - 1) + min) / AUDIO_SAMPLES;
  maxAudioAvg = (maxAudioAvg * (AUDIO_SAMPLES - 1) + max) / AUDIO_SAMPLES;
}

// void leftWingLinearUpto(uint8_t x, CRGB c)
// {
//   leds[lerp8by8(LW_FRONT_START, LW_FRONT_PEAK, x)] += c;
//   leds[lerp8by8(LW_MIDDLE_START, LW_MIDDLE_PEAK, x)] += c;
//   leds[lerp8by8(LW_MIDDLE_END, LW_MIDDLE_PEAK, x)] += c;

//   if (float(x) > LW_LERP_OFFSET)
//   {
//     uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
//     leds[lerp8by8(LW_BACK_START, LW_BACK_PEAK, xx)] += c;
//     leds[lerp8by8(LW_BACK_END, LW_BACK_PEAK, xx)] += c;
//   }
// }

void betterAudioSetup()
{
  currentDelay = 2; // 500Hz
  shouldClear = false;
  usePixies = false;
}

void betterAudio()
{
  fadeToBlackBy(leds, NUM_LEDS, 7);

  // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  static uint8_t hue = 0;
  static float lastPeak = 0;
  hue++;

  hue %= 128;
  CRGB c = CRGB::Purple;
  CRGB c2 = CRGB::Red;
  bodyBack.fill(c.nscale8_video(10));
  bodyFront.fill(c.nscale8_video(10));
  headLeft.fill(c2.nscale8_video(30));
  headRight.fill(c2.nscale8_video(30));
  // headLeft.fill(c);
  // headRight.fill(c);

  if (audioPeak.available())
  {
    float peak = audioPeak.read();
    audioUpdate(peak);

    if (peak < minAudioAvg)
    {
      peak = minAudioAvg;
    }

    int p = 128.0 * (peak - minAudioAvg) / (maxAudioAvg - minAudioAvg);

    wingLinearLerpTo(p, CHSV(HUE_BLUE + p, 230, p));

    if (p > 220)
    {
      lwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
      rwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
    }

    // lwFrontInner.lerpTo(p, CHSV(HUE_BLUE + p, 230, p));
    // lwMiddleTop.lerpTo(p, CHSV(HUE_BLUE + p, 230, p));
    // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    // rightWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
    // bodyFront(p, CHSV(HUE_BLUE + p, 230, p));
    // bodyBack(p, CHSV(HUE_BLUE + p, 230, p));

    // if (p > 210)
    // {
    //   fill(LW_BACK_PEAK, LW_BACK_END + 20, CRGB::Yellow);
    // }

    // if (p > 190)
    // {
    //   eyes.setPixelColor(0, 155, 0, 0);
    //   fiberTail.setPixelColor(0, 0, 255, 0);
    //   fiberHead.setPixelColor(0, 0, 0, 255);
    //   eyes.setBrightness(50);
    //   fiberTail.setBrightness(p);
    //   fiberHead.setBrightness(p);

    //   // all three must be shown in this order - otherwise it will flicker
    //   fiberTail.show();
    //   fiberHead.show();
    //   eyes.show();
    // }

    // if (peak > lastPeak)
    // {
    //   lastPeak = peak;
    // }
    // // leftWingLinear(peak * 255.0, CHSV(255, 255, 255));
    // // rightWingLinear(peak * 255.0, CHSV(255, 255, 255));

    // lastPeak *= 0.99;
  }
}

void audioUpdate2(float sample)
{
  float min, max;

  if (sample < MIN_AUDIO_LEVEL)
    sample = 0;

  audioSamples[audioSampleIdx] = sample;
  if (++audioSampleIdx >= AUDIO_SAMPLES)
    audioSampleIdx = 0;

  min = max = audioSamples[0];
  for (uint8_t i = 1; i < AUDIO_SAMPLES; i++)
  {
    if (audioSamples[i] < min)
      min = audioSamples[i];
    else if (audioSamples[i] > max)
      max = audioSamples[i];
  }
  if ((max - min) < MIN_DIST_AUDIO_LEVELS)
    max = min + MIN_DIST_AUDIO_LEVELS;
  minAudioAvg = (minAudioAvg * (AUDIO_SAMPLES - 1) + min) / AUDIO_SAMPLES;
  maxAudioAvg = (maxAudioAvg * (AUDIO_SAMPLES - 1) + max) / AUDIO_SAMPLES;
}

elapsedMicros audioSamplingTimer;
elapsedMillis audioShowTimer;
boolean beatDetected = false;

AudioAnalyzeRMS rms;
// AudioConnection patchCord2(adc1, rms);

AudioAnalyzeFFT1024 fft;
AudioConnection patchCord3(adc1, fft);

#define AUDIO_SAMPLES 100
int sampleCount = 0;

#define BEAT_BANDS 30
#define LONGTERM_SAMPLES 120 // ~1s
#define SHORTTERM_SAMPLES 2
#define BEAT_COUNTER_SAMPLES 400
#define BEAT_SAMPLES 100

#define CUTOFF 3
// #define DELTA_SAMPLES 300  // 3s
#define DELTA_SAMPLES 150

#define MAX_TIME 200

float deltaSamples[BEAT_BANDS][DELTA_SAMPLES];
float deltas[BEAT_BANDS];
float longtermSamples[BEAT_BANDS][LONGTERM_SAMPLES];
float avgSamples[LONGTERM_SAMPLES];
float delta[BEAT_BANDS];
float longtermAvg[BEAT_BANDS];

float shorttermSamples[BEAT_BANDS][SHORTTERM_SAMPLES];

float band[BEAT_BANDS];

int count[BEAT_BANDS];
int beatSpread[MAX_TIME];
float c[BEAT_BANDS];
int beatCounter[BEAT_COUNTER_SAMPLES];
int beatsArray[BEAT_SAMPLES];

int bandMap[BEAT_BANDS][2] = {
    {0, 0},
    {1, 1},
    {2, 2},
    {3, 4},
    {5, 6},
    {7, 8},
    {9, 10},
    {11, 13},
    {14, 16},
    {17, 20},
    {21, 24},
    {25, 29},
    {30, 35},
    {36, 42},
    {43, 50},
    {51, 59},
    {60, 70},
    {71, 82},
    {83, 96},
    {97, 112},
    {113, 131},
    {132, 153},
    {154, 178},
    {179, 207},
    {208, 241},
    {242, 280},
    {281, 326},
    {327, 379},
    {380, 440},
    {441, 511}};

float bandCorrection[BEAT_BANDS] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0};
// 0,
// 0.02,
// 0.01,
// 0.01,
// 0.08,
// 0.02,
// 0.02};

float beatsGlobalAvg;
int longPos = 0;
int shortPos = 0;
int deltaPos = 0;
int beatCounterPos = 0;
int nextBeatCounter = 0;
int beat = 0;
int beatPos = 0;
float threshold = 0;
float predictiveInfluenceConstant = .1;
float predictiveInfluence;
int cyclePerBeatIntensity;
float standardDeviation;
int cyclesPerBeat;

void newAudioSetup()
{
  currentDelay = 0;
  // shouldClear = true;
  shouldClear = false;
  shouldShow = true;
  usePotentiometer = 0;
  usePixies = 1;

  FastLED.setBrightness(255);

  for (int i = 0; i < BEAT_BANDS; i += 1)
  {
    count[i] = 1;
    longtermAvg[i] = 0;
    delta[i] = 0;
    c[i] = 1.5;
  }
}

// 60 fps =>  16-17ms
// our fps is => 86 => 11-12ms

elapsedMicros fps0;

void newAudio()
{
  static int fb = 0;

  fadeToBlackBy(leds, NUM_LEDS, 5);

  if (fft.available())
  {
    // return;
    // Serial.println(fps0);
    // int old = fps0;
    // fps0 = 0;
    // Serial.print(" ");

    longPos++;
    if (longPos >= LONGTERM_SAMPLES)
      longPos = 0;
    shortPos++;
    if (shortPos >= SHORTTERM_SAMPLES)
      shortPos = 0;
    deltaPos++;
    if (deltaPos >= DELTA_SAMPLES)
      deltaPos = 0;
    beatPos++;
    if (beatPos >= BEAT_SAMPLES)
      beatPos = 0;

    float currentAvg = 0;
    // for (int i = 0; i < BEAT_BANDS; i++)
    // {

    //   Serial.print(float(i), i > 9 ? 1 : 2);
    //   Serial.print(" ");
    // }
    // Serial.println("");

    for (int i = 0; i < BEAT_BANDS; i++)
    {

      float v = fft.read(bandMap[i][0], bandMap[i][1]);

      // if (v < 0.01)
      //   v = 0;

      shorttermSamples[i][shortPos] = v;

      // band scaling due to noise
      // v = constrain(v - bandCorrection[i], 0, 2.0);

      currentAvg += v;

      longtermAvg[i] = 0;
      for (int j = 0; j < LONGTERM_SAMPLES; j++)
      {
        longtermAvg[i] += longtermSamples[i][j];
      }

      band[i] = 0;
      for (int j = 0; j < SHORTTERM_SAMPLES; j++)
      {
        band[i] += shorttermSamples[i][j];
      }
      band[i] = band[i] / float(SHORTTERM_SAMPLES);

      longtermSamples[i][longPos] = v;
      // if (band[i] >= 0.001)
      // {
      //   Serial.print(band[i], 4);
      //   Serial.print(" ");
      // }
      // else
      // {
      //   Serial.print("  -  ");
      // }
    }

    currentAvg = currentAvg / float(BEAT_BANDS);
    // Serial.println("");
    // return;
    float con = fmap(potentiometer, 0, 65535, 1.0, 4.0);
    float mul = fmap(potentiometer, 0, 65535, -10.0, -170.0);
    con = con - 0.1;

    // Serial.print(mul, 4);
    // Serial.println(" ");
    // Serial.println(con, 4);
    // Serial.println("");

    for (int i = 0; i < BEAT_BANDS; i++)
    {
      delta[i] = 0.0;
      longtermAvg[i] = longtermAvg[i] / float(LONGTERM_SAMPLES);

      // store a delta between the current value and the longterm avg per band
      deltaSamples[i][deltaPos] = pow(longtermAvg[i] - band[i], 2);

      // calculate the per band delta avg for all samples we have
      for (int j = 0; j < DELTA_SAMPLES; j += 1)
      {
        delta[i] += deltaSamples[i][j];
      }

      // Serial.println();
      delta[i] = delta[i] / float(DELTA_SAMPLES);

      c[i] = (mul * delta[i]) + con;
      // c[i] = (-100 * delta[i]) + 2.5;
      // c[i] = (-60 * delta[i]) + 1.9;
      // c[i] = (-60 * delta[i]) + 1.9;
      // c[i] = (-50 * delta[i]) + 1.5;

      // static float maxDelta = 0;

      // per band delta avg : 0.00775
      // float dx = fmap(constrain(delta[i], 0, 0.1), 0, 0.1, 0, .4);
      // float dx = constrain(fmap(delta[i], 0, 1, 0, .4), 0, .4);
      // lt avg ~ 0.3
      // float lx = fmap(constrain(pow(longtermAvg[i], .5), 0, 6), 0, 20, .3, 0);
      // float lx = 0;
      // float count1 = fmap(constrain(count[i], 0, 15), 0, 15, 1.0, 0.0);
      // float count2 = fmap(constrain(count[i], 30, 200), 30, 200, 0.0, .75);

      // c[i] = 1.3 + dx + lx + count1 - count2;
      // c[i] = 1.3 + dx + lx + count1 - count2;
      // Serial.print(i);
      // Serial.print(" count:");
      // Serial.print(count[i]);
      // Serial.print(" delta:");
      // Serial.print(delta[i], 5);
      // Serial.print(" dx:");
      // Serial.print(dx, 5);
      // Serial.print(" lx:");
      // Serial.print(lx, 5);
      // Serial.print(" c1:");
      // Serial.print(count1, 5);
      // Serial.print(" c2:");
      // Serial.print(-count2, 5);
      // Serial.print(" prepre:");
      // Serial.print(c[i], 5);

      // if (cyclePerBeatIntensity / standardDeviation > 3.5)
      // {
      //   predictiveInfluence = predictiveInfluenceConstant * (1 - cos((float(nextBeatCounter) * TWO_PI) / float(cyclesPerBeat)));
      //   predictiveInfluence *= fmap(constrain(cyclePerBeatIntensity / standardDeviation, 3.5, 20), 3.5, 15, 1, 6);
      //   if (cyclesPerBeat > 10)
      //     c[i] = c[i] + predictiveInfluence;
      // }
      // Serial.print(" => ");
      // Serial.print(c[i], 5);
      // Serial.println();

      // c[i] = 1.5;
    }
    // Serial.println("");
    // return;

    // avg over some frequency + calc global avg LONGTERM_SAMPLES
    // one bin is 43 HZ -> 18275hz
    avgSamples[longPos] = fft.read(0, 425) / 425.0;
    float globalAvg = 0;
    for (int j = 0; j < LONGTERM_SAMPLES; j += 1)
      globalAvg += avgSamples[j];
    globalAvg = globalAvg / float(LONGTERM_SAMPLES);

    // Serial.print("global: ");
    // Serial.println(globalAvg, 8);

    if (globalAvg < 0.0005)
      return;

    // min 0.00042968
    // max from laptop 0.00277068
    // need to find real max

    beat = 0;

    for (int i = 0; i < BEAT_BANDS; i++)
    {
      // Serial.print(delta[i],4);
      // Serial.print(delta[i]/globalAvg,4);
      // Serial.print(" ");
    }
    // Serial.println();
    // for (int i = 0; i < BEAT_BANDS; i++)
    // {

    //   Serial.print(float(i), i > 9 ? 1 : 2);
    //   Serial.print(" ");
    // }
    // Serial.println("");

    // find

    for (int i = 0; i < BEAT_BANDS; i += 1)
    {
      // Serial.print(band[i],3);
      // Serial.print(">");
      // Serial.print(longtermAvg[i],3);
      // Serial.print("=");
      // Serial.print(c[i],4);
      // Serial.print(" ");
      //&& delta[i] > 0.0005
      if (band[i] > longtermAvg[i] * c[i])
      // if (band[i] > longtermAvg[i] * 1.5 && count[i] > 7)
      {

        if (false)
        {
          Serial.print("beat in ");
          Serial.print(i);
          Serial.print(" band:");
          Serial.print(band[i], 6);
          Serial.print(" avg:");
          Serial.print(longtermAvg[i], 6);
          Serial.print(" c:");
          Serial.print(c[i], 6);
          Serial.print(" avg*c: ");
          Serial.print(longtermAvg[i] * c[i], 6);
          Serial.print(" count:");
          Serial.print(count[i]);
          Serial.println();
        }

        // Serial.print("  x  ");
        // if (count[i] > 12 & count[i] < 200)
        // {
        //   beatCounter[beatCounterPos % BEAT_COUNTER_SAMPLES] = count[i];
        //   beatCounterPos += 1;
        // }
        count[i] = 0;
      }
      else
      {
        // Serial.print("  -  ");
      }
    }

    // Serial.println("");

    for (int i = 0; i < BEAT_BANDS; i += 1)
      if (count[i] == 0)
        beat += 1;

    // beatsArray[beatPos] = beat;
    // for (int i = 0; i < BEAT_SAMPLES; i += 1)
    //   beatsGlobalAvg += beatsArray[i];
    // beatsGlobalAvg = beatsGlobalAvg / float(BEAT_SAMPLES);

    // c[0] = 1.5 + fmap(constrain(nextBeatCounter, 0, 5), 0, 5, 5, 0);
    // c[0] = 2.0 + map(constrain(nextBeatCounter, 0, 5), 0, 5, 5, 0);
    // c[0] = 1.5; //+ map(constrain(nextBeatCounter, 0, 5), 0, 5, 5, 0);
    // c[0] = 1.5 + map(constrain(nextBeatCounter, 0, 5), 0, 5, 5, 0);

    // if (cyclesPerBeat > 10)
    //   c[0] = c[0] + .75 * (1 - cos((float(nextBeatCounter) * TWO_PI) / float(cyclesPerBeat)));

    // global avg = 0.00277719

    // threshold = constrain(c[0] * beatsGlobalAvg + fmap(constrain(globalAvg, 0, 2), 0, 2, 4, 0), 5, 1000);
    // threshold = constrain(c[0] * beatsGlobalAvg, 5, 1000);
    // if (false)
    // {
    //   Serial.print("c0: ");
    //   Serial.print(c[0], 8);
    //   Serial.print(" cyclesPerBeat: ");
    //   Serial.print(cyclesPerBeat, 8);
    //   Serial.print(" nextBeatCounter: ");
    //   Serial.print(nextBeatCounter);
    //   Serial.print(" beatsGlobalAvg: ");
    //   Serial.print(beatsGlobalAvg, 8);
    //   Serial.print(" globalAvg: ");
    //   Serial.print(globalAvg, 8);
    //   Serial.print(" beats: ");
    //   Serial.print(beat);
    //   Serial.print(" threshold: ");
    //   Serial.print(threshold, 8);
    //   Serial.println("");
    // }

    threshold = 5;

    // Serial.println(beat);
    if (beat > threshold && nextBeatCounter > 5)
    {
      // float d = currentAvg - globalAvg;
      // int p = constrain(d * 25500.0, 0, 255);

      if (false)
      {
        Serial.print("BEAT c0: ");
        Serial.print(c[0], 8);
        Serial.print(" globalAvg: ");
        Serial.print(globalAvg, 8);
        Serial.print(" beats: ");
        Serial.print(beat);
        Serial.print(" threshold: ");
        Serial.print(threshold, 8);
        Serial.print(" nextBeat: ");

        Serial.print(" total beat avg=");
        Serial.print(beatsGlobalAvg, 8);

        Serial.print(" currentAvg: ");
        Serial.print(currentAvg, 8);
        // Serial.print(" d=");
        // Serial.print(d, 6);
        // Serial.print(" P=");
        // Serial.print(p);
        Serial.println("");
      }

      int hue = beatsin8(15, 0, 255);
      CRGB c = CHSV(hue, 255, 255);
      fiberTail.setPixelColor(0, c.r, c.g, c.b);
      eyes.setPixelColor(0, c.r, c.g, c.b);
      fb = 255;

      // fadeLightBy(leds, NUM_LEDS, 100);
      // blur1d(leds, NUM_LEDS, 100);
      // lwFrontInner.lerpTo(150, CHSV(HUE_RED, 230, 255));
      // rwFrontInner.lerpTo(150, CHSV(HUE_RED, 230, 255));
      // for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
      // {
      //   leds[i].setHSV(0, 255, 255);
      // }
      nextBeatCounter = 0;
    }

    fb--;
    fb--;
    if (fb <= 0)
      fb = 0;

    fiberTail.setBrightness(fb);
    fiberTail.show();

    if (buttonState[EYE_STATE] == 1)
    {
      eyes.setBrightness(fb);
      eyes.show();
    }

    for (int i = 0; i < BEAT_BANDS; i += 1)
      if (count[i] == 0)
      {
        uint8_t h = beatsin8(15, 0, 255);
        int hue = beatsin8(15, 0, 255) + i * 9;

        int from = map(i, 0, BEAT_BANDS, 0, 255);
        int to = map(i + 1, 0, BEAT_BANDS, 0, 255);
        CRGB c = CHSV(hue, 255, 255);

        leftWingLinearLerpFromTo(from, to, c);
        rightWingLinearLerpFromTo(from, to, c);

        bodyBack.lerpFromTo(from, to, c);
        bodyFront.lerpFromTo(from, to, c);

        headLeft.lerpFromTo(from, to, c);
        headRight.lerpFromTo(from, to, c);

        fiberLeft.lerpFromTo(from, to, c);
        fiberRight.lerpFromTo(from, to, c);

        // rwFront.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
        // rwBackBottom.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
        // rwBackTop.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));

        // rwMiddleTop.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
        // rwMiddleBottom.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));

        // bodyBack.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
        // bodyFront.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));

        // headLeft.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
        // headRight.lerpFromTo(map(i, 0, BEAT_BANDS, 0, 255), map(i + 1, 0, BEAT_BANDS, 0, 255), CHSV(hue, 255, 255));
      }

    // for (int i = 0; i < MAX_TIME; i++)
    //   beatSpread[i] = 0;
    // for (int i = 0; i < BEAT_COUNTER_SAMPLES; i++)
    // {
    //   beatSpread[beatCounter[i]] += 1;
    // }

    // cyclesPerBeat = amode(beatCounter, BEAT_COUNTER_SAMPLES);
    // if (cyclesPerBeat < 20)
    //   cyclesPerBeat *= 2;

    // cyclePerBeatIntensity = amax(beatSpread, MAX_TIME);

    // standardDeviation = 0;
    // for (int i = 0; i < MAX_TIME; i++)
    //   standardDeviation += pow(BEAT_COUNTER_SAMPLES / MAX_TIME - beatSpread[i], 2);
    // standardDeviation = pow(standardDeviation / MAX_TIME, .5);

    for (int i = 0; i < BEAT_BANDS; i += 1)
      count[i] += 1;

    // deltaPos += 1;
    nextBeatCounter += 1;
    beatPos += 1;

    // Serial.print(audioSamplingTimer);
    // audioSamplingTimer = 0;
    // Serial.print("|FFT: ");
    // for (int i = 0; i < 20; i++)
    // {
    //   float n = fft.read(i);
    //   if (n >= 0.01)
    //   {
    //     Serial.print(n);
    //     Serial.print(" ");
    //   }
    //   else
    //   {
    //     Serial.print("  -  ");
    //   }
    // }
    // Serial.println();
  }

  // EVERY_N_MILLISECONDS(1000)
  // {
  //   Serial.print("fft=");
  //   Serial.print(fft.processorUsage());
  //   Serial.print(",");
  //   Serial.print(fft.processorUsageMax());
  //   Serial.print(" Memory: ");
  //   Serial.print(AudioMemoryUsage());
  //   Serial.print(",");
  //   Serial.print(AudioMemoryUsageMax());
  //   Serial.println("");
  // }

  // // a sample is ready roughly every 3ms - lets test that assumption
  // if (rms.available())
  // {
  //   float peak = rms.read();
  //   Serial.println("peak " + String(peak) + " t=" + String(audioSamplingTimer));
  //   audioSamplingTimer = 0;
  //   sampleCount++;

  //   if(sampleCount >= AUDIO_SAMPLES) {

  //   }
  // }

  // a sample is ready roughly every 3ms - lets test that assumption
  // if (audioSamplingTimer > 22)
  // {
  //   if (audioPeak.available())
  //   {
  //     audioSamplingTimer = 0;
  //     float peak = audioPeak.read();

  //   }
  // }
  // if (audioShowTimer > 10)
  // {
  //   audioShowTimer = 0;

  //   lwFrontInner.lerpTo(p, CHSV(HUE_BLUE, 230, p));
  // }
  // fadeToBlackBy(leds, NUM_LEDS, 5);

  // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  // if (audioPeak.available())
  // {
  //   float peak = audioPeak.read();
  //   audioUpdate(peak);

  //   if (peak < minAudioAvg)
  //   {
  //     peak = minAudioAvg;
  //   }

  //   int p = 128.0 * (peak - minAudioAvg) / (maxAudioAvg - minAudioAvg);

  //   lwFrontInner.lerpTo(p, CHSV(HUE_BLUE, 230, p));
  //   rightWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
  // }
}

void heartbeatSetup()
{
  currentDelay = 10;
  usePotentiometer = 0;
  usePixies = 1;
  shouldClear = false;
  clear();
  FastLED.show();
  FastLED.setBrightness(255);
}

elapsedMillis timeElapsed;

void heartbeat()
{
  static int fadeSpeed = 5;
  fadeToBlackBy(leds, NUM_LEDS, fadeSpeed);

  leds[fiberleft(0)] = CRGB(255, 0, 0);
  leds[fiberleft(1)] = CRGB(255, 0, 0);
  leds[fiberleft(2)] = CRGB(255, 0, 0);
  leds[fiberleft(3)] = CRGB(255, 0, 0);
  leds[fiberright(0)] = CRGB(255, 0, 0);
  leds[fiberright(1)] = CRGB(255, 0, 0);
  leds[fiberright(2)] = CRGB(255, 0, 0);
  leds[fiberright(3)] = CRGB(255, 0, 0);

  static int b = 0;

  // uint8_t b = beat8(60);
  // if (b < 40)
  // {
  //   b = 40;
  // }
  // fill(BODY_FRONT_START, BODY_FRONT_END, CHSV(0, 230, 40));
  if (timeElapsed > 1000)
  {
    timeElapsed = 0;
    fadeSpeed = 15;
    b = 1;
    for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
    {
      leds[i].setHSV(0, 255, 255);
    }
  }
  if (timeElapsed > 200 && b == 1)
  {
    // timeElapsed = 0;
    fadeSpeed = 10;
    b = 0;
    for (int i = BODY_FRONT_START + 1; i < BODY_FRONT_END - 9; i++)
    {
      leds[i].setHSV(0, 255, 255);
    }
  }

  int eyebrightness = map(potentiometer, 0, 65535, 0, 220);

  eyes.setPixelColor(0, 255, 0, 0);
  eyes.setBrightness(beatsin8(15, eyebrightness, 35 + eyebrightness));
  eyes.show();

  fiberHead.setPixelColor(0, 240, 10, 10);
  fiberHead.setBrightness(150);
  fiberHead.show();

  fiberTail.setPixelColor(0, 255, 0, 0);
  fiberTail.setBrightness(255);
  fiberTail.show();
}

// the loop routine runs over and over again forever:
void loop()
{

  if (shouldClear)
    clear();

  if (!usePixies)
    basicPowerLedMode();

  modes[currentMode][0]();

  if (shouldShow)
    FastLED.show();

  EVERY_N_MILLISECONDS(250) { checkPotentiometer(); }
  if (PAD_CONNECTED)
  {
    EVERY_N_MILLISECONDS(100) { checkButtons(); }
    // EVERY_N_MILLISECONDS(30) { checkButtons(); }
  }
  EVERY_N_MILLISECONDS(500) { testled(); }
  EVERY_N_MILLISECONDS(100) { checkSerial(); }

  // EVERY_N_MILLISECONDS(250) { reportSensor(); }
  if (currentDelay > 0)
    delay(currentDelay);
}
