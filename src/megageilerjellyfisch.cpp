// effect ideas
// - heart beats
// rotations acceleration
// #define FASTLED_ALLOW_INTERRUPTS 0
// #define USE_OCTOWS2811
// #include <OctoWS2811.h>
#include <Arduino.h>
#include <Audio.h>
#include <ADC.h>
#include <SoftwareSerial.h>
#include "FastLED.h"
#include <Adafruit_Pixie.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoTrellis.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <RCSwitch.h>
#include <vector>

float FindE(int bands, int bins);

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define POTENTIOMETER_PIN A17
#define MIC_PIN A2
#define REMOTE_RECEIVER_PIN 24

// analog - digital
ADC *adc = new ADC();

// remote codes
#define REMOTE_BUTTON_A 240191
#define REMOTE_BUTTON_B 4382
#define REMOTE_BUTTON_C 4242
#define REMOTE_BUTTON_D 987654

// remote switch 433 Mhz
RCSwitch remoteSwitch = RCSwitch();

// motion sensor
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire2);

// button board
Adafruit_NeoTrellis pad(&Wire1);
#define PAD_CONNECTED true

// active or not active for the pad buttons
uint8_t buttonState[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t buttonStateful[16] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t buttonDebounce[16] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};

// 3W pixels
// SoftwareSerial pixieSerial(-1, 26);
// Adafruit_Pixie pixie = Adafruit_Pixie(1, &Serial1);

#define NUM_LEDS 960
#define NUM_LEDS_PER_STRIP 240
#define NUM_COIL_LEDS 77
#define NUM_SEGMENTS 6
#define NUM_LEDS_PER_SEGMENT 60
#define NUM_FIBERS 7
#define NUM_STRIPS 4
#define NUM_FISH_LEDS 12
#define MAX_BRIGHTNESS 150
#define TEENSY_LED 13

#define NUM_BIG_FISH_LEDS 12
#define NUM_MEDIUM_FISH_LEDS 10
#define NUM_SMALL_FISH_LEDS 9

// 2,14,7,8  ,6,20,21,5
CRGB powerLeds[2];
CRGB leds[NUM_LEDS];
CRGB fiberLeds[NUM_FIBERS];
CRGB fishLeds[12];

// we map the fiberLeds to fiberLedsNeoPixel
Adafruit_NeoPixel fiberLedsNeoPixel(7, 21, NEO_GRBW + NEO_KHZ800);

AudioInputAnalog adc1(MIC_PIN);
AudioAnalyzePeak peak1;
AudioConnection patchCord1(adc1, peak1);

#include "segment.h"

typedef void (*Mode[2])(void);

// buttons
#define EYE_STATE 15

// global state
int8_t currentMode = 0;
int8_t previousMode = 0;
int8_t nextScheduledMode = -1;
int8_t nextScheduledModeDir = 0;
uint8_t currentBrightness = MAX_BRIGHTNESS;
uint16_t currentDelay = 0; // delay between frames
uint8_t shouldClear = 1;   // clear all leds between frames
uint8_t shouldShow = 1;
uint8_t usePixies = 0;                       // pixies on
uint8_t potentiometerControlsBrightness = 1; // use potentiometer to get brightness value for next frame
uint8_t useFibers = 1;
uint8_t scheduleStrobo = 0;
// Debug stuff to count LEDS properly
static int globalP = 0;

int accelTestMV = 0;

elapsedMillis fps0;

class Effect
{
public:
  virtual void setup(void){};
  virtual void draw(void){};
};

std::vector<Effect *> effects;

#include "inc/util.cpp"

TrellisCallback readButton(keyEvent evt)
{
  Serial.print("BUTTON: ");
  Serial.print(evt.bit.NUM);

  if (buttonStateful[evt.bit.NUM])
  {
    Serial.print(" OLDSTATE=");
    Serial.print(buttonState[evt.bit.NUM]);
    if (buttonState[evt.bit.NUM] && evt.bit.EDGE == SEESAW_KEYPAD_EDGE_FALLING && millis() > buttonDebounce[evt.bit.NUM] + 1000)
    {
      buttonState[evt.bit.NUM] = 0;
      pad.pixels.setPixelColor(evt.bit.NUM, 0);
      buttonDebounce[evt.bit.NUM] = millis();
    }
    else if (!buttonState[evt.bit.NUM] && evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING && millis() > buttonDebounce[evt.bit.NUM] + 1000)
    {
      buttonState[evt.bit.NUM] = 1;
      CRGB c = CHSV(map(evt.bit.NUM, 0, pad.pixels.numPixels(), 0, 255), 240, 240);
      pad.pixels.setPixelColor(evt.bit.NUM, pad.pixels.Color(c.r, c.g, c.b));
      buttonDebounce[evt.bit.NUM] = millis();
    }
    Serial.print(" NEWSTATE=");
    Serial.print(buttonState[evt.bit.NUM]);
  }
  else
  {
    if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING)
    {
      CRGB c = CHSV(map(evt.bit.NUM, 0, pad.pixels.numPixels(), 0, 255), 240, 240);
      pad.pixels.setPixelColor(evt.bit.NUM, pad.pixels.Color(c.r, c.g, c.b));

      switch (evt.bit.NUM)
      {
        // new FiberAndFishsEffect(), // 0
        // new Fibercrazy(), // 1
        // new Heartbeat(), // 2
        // new AccelTest(), // 3
        // new NewAudio(), // 4
        // new JellyAudio(), // 5
        // new JellyColorEffect(), // 6
        // new RunnersEffect(), // 7
        // new ColorWheelNew(), // 8
        // new ColorWheelWithSparkels(), // 9
        // new Streaming(), // 10
        // new Rings(), // 11
        // new Juggle(), // 12
        // // new DropEffect(),
        // new Bouncy(), // 13
        // new PrideEffect(), // 14
        // new IceSparkEffect(), // 15
        // new SinelonEffect(), // 16
        // new PixelFiringEffect(), // 17
        // // new BlurEffect(),
        // new PaletteTestEffect()}; // 18

      case 0:
        nextScheduledMode = 0;
        break;
      case 1:
        nextScheduledMode = 2;
        break;
      case 2:
        nextScheduledMode = 4;
        break;
      case 4:
        nextScheduledMode = 8;
        break;
      case 5:
        nextScheduledMode = 10;
        break;
      case 6:
        nextScheduledMode = 12;
        break;
      case 8:
        nextScheduledMode = 13;
        break;
      case 9:
        nextScheduledMode = 14;
        break;
      case 10:
        nextScheduledMode = 15;
        break;
      case 12:
        nextScheduledMode = 16;
        break;
      case 13:
        nextScheduledMode = 18;
        break;
      case 14:
        scheduleStrobo = 1;
        break;
      case 11:
        nextScheduledModeDir = -1;
        break;
      case 15:
        nextScheduledModeDir = 1;
        break;
      }
    }
    else if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_FALLING)
    {
      pad.pixels.setPixelColor(evt.bit.NUM, 0);
    }
  }
  Serial.println("");
  pad.pixels.show();
  return 0;
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

void baselineEyes()
{
  if (!buttonState[3])
  {
    powerLeds[0] = CRGB::Black;
    powerLeds[1] = CRGB::Black;
    return;
  }

  powerLeds[0] = CHSV(0, 255, 120);
  powerLeds[1] = CHSV(0, 255, 120);
}

void baselineFishAndFibers()
{
  baselineEyes();
  if (!buttonState[7])
  {
    for (int i = 0; i < NUM_FIBERS; i++)
    {
      fiberLeds[i] = CRGB::Black;
    }
    for (int i = 0; i < NUM_FISH_LEDS; i++)
    {
      fishLeds[i] = CRGB::Black;
    }

    return;
  }

  for (int i = 0; i < NUM_FIBERS; i++)
  {
    fiberLeds[i] = CHSV(i * 30 + beat8(5), i + beat8(5), 255);
  }
  for (int i = 0; i < NUM_FISH_LEDS; i++)
  {
    fishLeds[i] = CHSV(i * 20 + beat8(5), i + beat8(5), 150);
  }
}

void checkSerial()
{
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'n')
      nextMode(1);
    else if (c == 'b')
      nextMode(-1);
    else
    {
      globalP++;
      Serial.print("POS:");
      Serial.println(globalP);
    }
  }
}

static uint16_t potentiometer = 0;

void checkPotentiometer()
{
  potentiometer = adc->analogRead(POTENTIOMETER_PIN, ADC_1);
  // Serial.print("POT: ");
  // Serial.println(potentiometer);
  // Serial.println(potentiometer * 3.3 / adc->getMaxValue(ADC_1), 2);
  // Serial.print("  ");
  if (potentiometerControlsBrightness)
  {
    LEDS.setBrightness(map(potentiometer, 0, 65535, 0, 255));
  }
}

void checkButtons()
{
  if (PAD_CONNECTED)
    pad.read();
}

void readRemoteSwitch()
{
  if (remoteSwitch.available())
  {

    int value = remoteSwitch.getReceivedValue();
    if (value != 0)
    {
      // nextMode(1);
      Serial.print("Received ");
      Serial.print(remoteSwitch.getReceivedValue());
      Serial.print(" / delay ");
      Serial.print(remoteSwitch.getReceivedDelay());

      static int codes[4] = {REMOTE_BUTTON_A, REMOTE_BUTTON_B, REMOTE_BUTTON_C, REMOTE_BUTTON_D};
      // for (int i = 0; i < 4; i++)
      // {
      //   if (codes[i] == value && PAD_CONNECTED)
      //   {
      //     pad.pixels.setPixelColor(12 + i, random8(), random8(), random8());
      //     pad.pixels.show();
      //   }
      // }

      // if (value == REMOTE_BUTTON_B)
      //   globalP++;
      if (value == REMOTE_BUTTON_A || value == REMOTE_BUTTON_B)
        nextMode(1);
      if (value == REMOTE_BUTTON_C || value == REMOTE_BUTTON_D)
        scheduleStrobo = 2;
    }
    remoteSwitch.resetAvailable();
  }
}

void runScheduluer()
{
  if (nextScheduledMode != -1)
  {
    setMode(nextScheduledMode);
    nextScheduledMode = -1;
  }
  if (nextScheduledModeDir)
  {
    nextMode(nextScheduledModeDir);
    nextScheduledModeDir = 0;
  }
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

void fill(uint16_t from, uint16_t to, CRGB c)
{
  fill_solid(leds + from, to - from, c);
}

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
  // if (ahrs.getOrientation(&orientation))
  // {
  //   Serial.print("Orientation: ");
  //   Serial.print(orientation.roll);
  //   Serial.print(" ");
  //   Serial.print(orientation.pitch);
  //   Serial.print(" ");
  //   Serial.print(orientation.heading);
  //   Serial.println("");

  //   float p0 = orientation.pitch - 90.0;

  //   Serial.print("p0: ");
  //   Serial.print(p0);
  //   Serial.println();

  //   if (p0 > 0)
  //   {
  //     bodyFront.lerpTo(p0 * 2, CRGB::Blue);
  //   }
  //   else
  //   {
  //     bodyBack.lerpToReverse(p0 * -2, CRGB::Blue);
  //   }
  // }

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
  // lsm.getEvent(&accel, &mag, &gyro, &temp);

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

  // wingLinearLerpTo(p, CHSV(HUE_BLUE + p, 230, p));

  // if (mv > 10)
  // {
  //   strobo();
  // }
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
      // eyes.setBrightness(255);
      // eyes.setPixelColor(0, 255, 255, 255);
      // eyes.show();
      state = On;
      timer = 0;
      c--;
      break;
    case NextOff:
      // eyes.setBrightness(0);
      // eyes.show();
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
  // lsm.getEvent(&accel, &mag, &gyro, &temp);

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
    // wingLinearLerp(i, c);
    // bodyFront.lerpAt(i, c);
    // bodyBack.lerpAt(255 - i, c);
    // leftWingLinear(i, CHSV(hue2 + 100, 255, 255));
    // rightWingLinear(i, CHSV(hue2 + 100, 255, 255));
    // bodyFront(i, CHSV(hue2 + 100, 255, 255));
    // bodyBack(i, CHSV(hue2 + 100, 255, 255));
  }

  if (mv > 10)
  {
    s.trigger();
    // headLeft.fill(CHSV(240, 240, 255));
    // headRight.fill(CHSV(240, 240, 255));
    // lwFrontOuter.fill(CHSV(240, 240, 255));
    // rwFrontOuter.fill(CHSV(240, 240, 255));
    // strobo();
  }

  s.advance();
}

void lerpTestSetup()
{
  currentDelay = 10;
  shouldClear = false;
}

void jellyModeSetup()
{
}

void jellyMode()
{
  fibers.fill(CHSV(120, 180, 180));
  for (int i = 0; i < NUM_LEDS_PER_SEGMENT; i++)
  {
    int h = lerp8by8(80, 160, beatsin8(10) + i);
    int s = lerp8by8(150, 250, beat8(7));
    int v = lerp8by8(200, 250, beatsin8(5));
    // inner.at(i, CHSV(h, 240, 240));
    inner.at(i, CHSV(h, s, v));
  }
}

int gPos = 0;

void segmentTest()
{

  globalP = globalP % 6;
  outerSegments[globalP].fill(CRGB::Red);
  innerSegments[globalP].fill(CRGB::Red);
  // inner.fill(CRGB::Red);
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

class Ring
{
public:
  int pos;
  int dir;
  int wait;

  Ring(int wait, int dir) //: dir(dir)
  {
    this->wait = wait;
    this->dir = dir;
    pos = 30;
  }
  void advance()
  {
    if (wait > 0)
    {
      wait--;
      return;
    }
    pos += dir;
    if (pos < 0 || pos >= 60)
    {

      pos = 30;
    }
  }
};

class Rings : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
    shouldClear = false;
  }

  void draw()
  {
    static elapsedMillis timer;
    if (timer < 20)
      return;
    timer = 0;
    CRGB c;
    static uint8_t idx = 0;
    if (idx > 4)
    {

      c = CRGB::Red;
      idx = 0;
      // c = CHSV(beatsin8(10), 180, 200);
    }
    else
    {
      idx++;
      c = CRGB::Black;
    }
    *outerSegments[0][30] = c;
    *outerSegments[0][29] = c;

    // for (int i = NUM_LEDS_PER_SEGMENT / 2 - 1; i > 0; i--)
    // {
    //   CRGB p = *outerSegments[0][i + 1];
    //   for (int j = 0; j < NUM_SEGMENTS; j++)
    //   {
    //     *outerSegments[j][i] = p;
    //     *innerSegments[j][i] = p;
    //   }
    // }
    for (int i = NUM_LEDS_PER_SEGMENT - 1; i > NUM_LEDS_PER_SEGMENT / 2; i--)
    {
      CRGB p = *outerSegments[0][i - 1];
      for (int j = 0; j < NUM_SEGMENTS; j++)
      {
        *outerSegments[j][i] = p;
        *innerSegments[j][i] = p;
      }
    }
  }
};

#include "palette.h"

class PaletteTestEffect : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
    currentDelay = 0;

    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
    SetupBlackAndWhiteStripedPalette();
  }
  void draw()
  {
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */

    uint8_t colorIndex = startIndex;

    // ChangePalettePeriodically();
    uint8_t brightness = 255;

    for (int i = 0; i < NUM_LEDS_PER_SEGMENT; i++)
    {
      for (int8_t j = 0; j < NUM_SEGMENTS; j++)
        outerSegments[j].at(i, ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending));
      colorIndex += 3;
    }
    for (int i = 0; i < NUM_COIL_LEDS; i++)
    {
      colorIndex += 2;
      coil.at(i, ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending));
    }
  }
};

class BlurEffect : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
    currentDelay = 0;
  }

  void draw()
  {
    if (random8() < 10)
    {
      uint8_t p = random8(NUM_LEDS_PER_SEGMENT - 2);
      uint8_t s = random8(NUM_SEGMENTS);
      outerSegments[s].at(p, CHSV(random8(), 200, 255));
      outerSegments[s].at(p + 1, CHSV(random8(), 255, 255));
      outerSegments[s].at(p + 2, CHSV(random8(), 200, 255));
    }

    static elapsedMillis t = 0;
    if (t < 20)
      return;
    t = 0;

    for (uint8_t i = 0; i < NUM_SEGMENTS; i++)
    {
      blur1d(outerSegments[i].ledP(), NUM_LEDS_PER_SEGMENT, 20);
    }
  }
};

class Pixel
{
public:
  float pos;
  float speed; // pixels per second
  elapsedMillis sleepTimer;
  int sleep;

  Pixel()
  {
    randomize();
    sleep = 0;
  }
  void randomize()
  {
    pos = 0;
    sleepTimer = 0;
    sleep = random16(1000) + 500;
    speed = (random8(6) + 1) / 4.0;
  }
  void advance()
  {
    pos += speed;
    if (pos > NUM_LEDS_PER_SEGMENT - 1)
    {
      randomize();
    }
  }
  bool sleeping()
  {
    return sleepTimer < sleep;
  }
};

#define NUM_FADE_PIXELS 10

class PixelFiringEffect : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
    potentiometerControlsBrightness = 0;
    FastLED.setBrightness(100);
    shouldClear = false;
  }

  void draw()
  {
    static elapsedMillis timer;
    if (timer < 2)
      return;
    timer = 0;

    static Pixel p[NUM_SEGMENTS];

    fadeUsingColor(leds, NUM_LEDS, CRGB(200, 150, 100));
    // fadeToBlackBy(leds, NUM_LEDS, 50);
    CRGB c = CRGB::White;
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
      if (!p[i].sleeping())
      {
        outerSegments[i].at(p[i].pos, c);
        innerSegments[i].at(p[i].pos, c);
        p[i].advance();
      }
    }
  }
};

class JellyAudio : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
  }

  void draw()
  {

    // for (int i = 0; i < 7; i++)
    // {
    //   fiberLeds[i] = CHSV(30 * i + beatsin8(10), 200, 255);
    //   // fiberLeds[i] = CRGB::Yellow; //CHSV(30 * i + beatsin8(10), 200, 255);
    //   strip.setPixelColor(i, to_rgbw(fiberLeds[i]));
    // }

    outer.fade(10);
    inner.fade(5);
    if (fps0 > 24)
    {
      if (peak1.available())
      {
        fps0 = 0;
        int monoPeak = peak1.read() * 200.0;

        int p = lerp8by8(0, NUM_LEDS_PER_SEGMENT, monoPeak);

        for (int i = 0; i < p; i++)
        {
          inner.at(i, CHSV(i * 4 + beat8(5), 200, 200 + i / 4));
          // outer.at(i, CHSV(i * 4 + beat8(5), 200, 200 + i / 4));
        }
      }
    }
  }
};

class Fibercrazy : public Effect
{
public:
  void setup()
  {
    shouldClear = true;
    useFibers = 1;
    currentDelay = 0;
  }

  void draw()
  {
    static elapsedMillis timer;
    if (timer < 2)
      return;
    timer = 0;

    uint8_t i = random8(NUM_FIBERS);
    fiberLeds[i] = CHSV(beatsin16(5) + i * 30, beat8(10), 255);
  }
};

#define NUM_BALLS 5
#define GRAVITY -1
#define h0 1

float vImpact0 = sqrt(-2 * GRAVITY * h0);

class Ball
{
public:
  float h;
  float tCycle;
  long tLast;
  int pos;
  float vImpact;
  float cor;
  uint8_t hue = 0;
  Ball()
  {
    tLast = millis() * 2;
    pos = 0;
    h = h0;
    vImpact = vImpact0;
    tCycle = 0;
    randomize();
  }
  void randomize()
  {
    cor = min(1.0, 0.2 + (random8(100, 200) / 200.0));
    hue = random8();
  }
  void advance(int t)
  {
    tCycle = t - tLast;
    h = 0.5 * GRAVITY * pow(tCycle / 1000, 2.0) + vImpact * tCycle / 1000;
    if (h < 0)
    {
      h = 0;
      vImpact = cor * vImpact;
      tLast = t;
      if (vImpact < 0.01)
      {
        vImpact = vImpact0;
        randomize();
      }
    }
    // insteadf of 60 pass it in
    pos = round(h * (60 - 1) / h0);
  }
};

class Bouncy : public Effect
{
public:
  void draw()
  {
    static Ball ball[NUM_BALLS];
    for (int i = 0; i < NUM_BALLS; i++)
    {
      ball[i].advance(millis() * 2);
      inner.atAdd(59 - ball[i].pos, CHSV(50 * i + ball[i].hue, 220, 200));
    }
  }
};

class Drop
{
public:
  float speed;
  CHSV color;
  float pos;
  Drop()
  {
    randomize();
  }
  void randomize()
  {
    pos = 0;
    speed = min((random8() / 100.0) + 0.5, 2.55);
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
    color.v = 100 + map(pos, 0, 255, 0, 255);
  }
};

class DropEffect : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
  }

  void draw()
  {

    static Drop drops[NUM_LEDS_PER_COIL_RING];

    fadeToBlackBy(coil.ledP(), coil.size, 10);
    // fadeToBlackBy(leds, NUM_LEDS, 20);

    // int p = beatsin8(5);
    // for (int i = 0; i < 5; i++)
    // {
    //   coil.lerpAt(max(p + i, 255), CHSV(beatsin8(5), 200, 200));
    // }

    EVERY_N_MILLISECONDS(100)
    {
      // for (int i = 0; i < NUM_LEDS_PER_COIL_RING; i++)
      // {
      //   pos[i] = (pos[i] + 1) % NUM_COIL_RINGS;
      // }
    }

    for (int i = 0; i < NUM_LEDS_PER_COIL_RING; i++)
    {
      drops[i].advance();
      int p = lerp8by8(0, NUM_COIL_RINGS, drops[i].pos);
      coil.at(i + (NUM_LEDS_PER_COIL_RING * p), drops[i].color);
      // coil.at(i + (NUM_LEDS_PER_COIL_RING * pos[i]), CHSV(i * 3 + beatsin8(20), 200, 200));
    }

    // int p = lerp16by8(0, 7, beat8(45));
    // for (int i = 0; i < 10; i++)
    // {
    //   coil.at(i * 7 + p, CRGB::BlueViolet);
    // }

    // blur1d(coil.ledP(), coil.size, 30);

    // for (int i = 0; i < 10; i++)
    // {
    //   coil.at(i * 7 + p, CRGB::BlueViolet);
    // }

    // for (int i = 0; i < 10; i++)
    // {
    //   innerSegments[globalP % 6].at(i + 30, CRGB::BlueViolet);
    //   outerSegments[globalP % 6].at(i + 30, CRGB::BlueViolet);
    // }

    // coil.at(globalP % 100, CRGB::BlueViolet);
  }
};

class PrideEffect : public Effect
{
public:
  void setup()
  {
    currentDelay = 50;
  }

  void draw()
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
};

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
  currentDelay = 0;
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
    speed = random8(10, 100) / 10.0f;
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
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
      innerSegments[i].lerpAt(pos, color);
    }
  }
};
#define WAVES 5
void newLerp()
{
  static elapsedMillis timer;
  if (timer < 20)
    return;
  timer = 0;
  fadeToBlackBy(leds, NUM_LEDS, 25);
  static Wave waves[WAVES];
  for (int i = 0; i < WAVES; i++)
  {
    waves[i].advance();
    waves[i].draw();
  }
}

void moveToOuterSetup()
{
  currentDelay = 3;
  shouldClear = false;
}

class ColorWheelNew : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
  }
  void draw()
  {
    // for (int i = 0; i < NUM_LEDS; i++)
    // {
    //   leds[i] = CHSV(5 * i + beat8(17 * 4), 240, 255);
    //   // leds[NUM_LEDS_PER_STRIP * (globalP % 8) + i] = CHSV(5 * i + beat8(17), 240, 255);
    // }
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
      for (int j = 0; j < 60; j++)
      {
        CRGB c = CHSV(5 * j + beat8(17), map8(beat8(1), 200, 255), 255);
        innerSegments[i].at(j, c);
        outerSegments[i].at(60 - j, c);
      }
    }
    for (int i = 0; i < NUM_COIL_LEDS; i++)
    {
      CRGB c = CHSV(5 * i + beat8(17), 240, map8(beat8(2), 20, 240));
      coil.at(i, c);
    }
    baselineFishAndFibers();
  }
};

class IceSparkEffect : public Effect
{
  void setup()
  {
    shouldClear = false;
  }

  void draw()
  {
    fadeToBlackBy(leds, NUM_LEDS, 128);

    static uint8_t sat = 0;
    sat++;
    for (int j = 0; j < 35; j++)
    {
      leds[random(0, NUM_LEDS)] = CHSV(170 + (random(20) - 10), random(255), 250);
    }
    baselineEyes();
  }
};

void sparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 10;
  potentiometerControlsBrightness = 0;
}

void sparks()
{
  for (int j = 0; j < 30; j++)
  {
    leds[random(0, NUM_LEDS)] = CRGB::White;
  }
}

void pixieTestSetup()
{
  usePixies = 1;
  currentDelay = 10;
  shouldClear = true;
}
void pixieTest()
{

  // eyes.setPixelColor(0, 255, 0, 0);
  // fiberTail.setPixelColor(0, 0, 255, 0);
  // fiberHead.setPixelColor(0, 0, 0, 255);
  // eyes.setBrightness(50);
  // fiberTail.setBrightness(0);
  // fiberHead.setBrightness(0);
  // // EVERY_N_MILLISECONDS(500)
  // // {

  // // all three must be shown in this order - otherwise it will flicker
  // fiberTail.show();
  // fiberHead.show();
  // eyes.show();
  // }
}

void rainbowSparksSetup()
{
  FastLED.setBrightness(255);
  currentDelay = 15;
  potentiometerControlsBrightness = 0;
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
  // lsm.getEvent(&accel, &mag, &gyro, &temp);
  // float mv = accel.acceleration.x + accel.acceleration.y + accel.acceleration.z + 9.81;
  float mv = accel.acceleration.x + (9.81 + 0.9);
  // Serial.println(mv);

  // if (mv > 10)
  // {
  //   strobo();
  // }

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

class Juggle : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
    shouldClear = false;
  }

  void draw()
  {
    static elapsedMillis timer = 0;
    if (timer < 20)
      return;
    timer = 0;
    static uint8_t gHue = 0;
    fadeToBlackBy(leds, NUM_LEDS, 10);
    byte dothue = 0;
    for (int i = 0; i < 8; i++)
    {
      leds[beatsin16(i + 3, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }
  }
};

class SinelonEffect : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
    FastLED.setBrightness(220);
    potentiometerControlsBrightness = false;
  }

  void draw()
  {
    static uint8_t gHue = 0;
    fadeToBlackBy(leds, NUM_LEDS, 20);

    // int pos = beatsin16(16, 0, NUM_LEDS);
    // int pos = beatsin16(40, 0, NUM_LEDS);
    int pos = beatsin16(map(potentiometer, 0, 65535, 2, 60), 0, NUM_LEDS);
    leds[pos] += CHSV(gHue, 255, 192);
    EVERY_N_MILLISECONDS(3) { gHue++; }

    baselineFishAndFibers();
  }
};

CRGB stroboC[5] = {CRGB(0, 0, 255), CRGB(255, 0, 0), CRGB(0, 255, 255), CRGB(255, 0, 255), CRGB(255, 255, 255)};

void strobo()
{

  for (int i = 0; i < 5; i++)
  {
    // delay(50);
    // eyes.setBrightness(255);
    // eyes.setPixelColor(0, 255, 255, 255);
    // // eyes.setPixelColor(0, 126, 6, 229);
    // // eyes.setPixelColor(0, stroboC[i].r, stroboC[i].g, stroboC[i].b);
    // eyes.show();
    // delay(50);
    // eyes.setBrightness(0);
    // eyes.show();
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
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setBrightness(255);
  // fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  // eyes.setBrightness(100);
  // eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  // eyes.show();

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
  potentiometerControlsBrightness = 0;
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
  // fiberTail.setPixelColor(0, c.r, c.g, c.b);
  // fiberTail.setBrightness(255);
  // fiberTail.show();

  // eyes
  // if (buttonState[EYE_STATE] == 1)
  // {
  byte ci = scale8(heat[NUM_LEDS_PER_STRIP + 40], 240);
  CRGB cc = ColorFromPalette(firePal, ci);

  // eyes.setBrightness(currentBrightness / 3);
  // eyes.setBrightness(100);
  // eyes.setPixelColor(0, cc.r, cc.g, cc.b);
  // }
  // else
  // {
  //   eyes.setBrightness(0);
  // }

  // eyes.show();

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
#include "inc/tests.cpp"

// AudioInputAnalog adc1(MIC_PIN);
// AudioAnalyzePeak audioPeak;
// AudioConnection patchCord1(adc1, audioPeak);

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

void betterAudioSetup()
{
  currentDelay = 2; // 500Hz
  shouldClear = false;
  usePixies = false;
}

void betterAudio()
{
  // fadeToBlackBy(leds, NUM_LEDS, 7);

  // // fill(HEAD_LEFT_START, HEAD_LEFT_END, CRGB::Green);
  // // fill(HEAD_RIGHT_START, HEAD_RIGHT_END, CRGB::Blue);

  // static uint8_t hue = 0;
  // static float lastPeak = 0;
  // hue++;

  // hue %= 128;
  // CRGB c = CRGB::Purple;
  // CRGB c2 = CRGB::Red;
  // // bodyBack.fill(c.nscale8_video(10));
  // // bodyFront.fill(c.nscale8_video(10));
  // // headLeft.fill(c2.nscale8_video(30));
  // // headRight.fill(c2.nscale8_video(30));
  // // // headLeft.fill(c);
  // // headRight.fill(c);

  // if (audioPeak.available())
  // {
  //   float peak = audioPeak.read();
  //   audioUpdate(peak);

  //   if (peak < minAudioAvg)
  //   {
  //     peak = minAudioAvg;
  //   }

  //   int p = 128.0 * (peak - minAudioAvg) / (maxAudioAvg - minAudioAvg);

  //   // wingLinearLerpTo(p, CHSV(HUE_BLUE + p, 230, p));

  //   if (p > 220)
  //   {
  //     // lwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  //     // rwFrontOuter.lerpTo(255, CHSV(HUE_BLUE + p, 230, p));
  //   }

  //   // lwFrontInner.lerpTo(p, CHSV(HUE_BLUE + p, 230, p));
  //   // lwMiddleTop.lerpTo(p, CHSV(HUE_BLUE + p, 230, p));
  //   // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
  //   // leftWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
  //   // rightWingLinear(p, CHSV(HUE_BLUE + p, 230, p));
  //   // bodyFront(p, CHSV(HUE_BLUE + p, 230, p));
  //   // bodyBack(p, CHSV(HUE_BLUE + p, 230, p));

  //   // if (p > 210)
  //   // {
  //   //   fill(LW_BACK_PEAK, LW_BACK_END + 20, CRGB::Yellow);
  //   // }

  //   // if (p > 190)
  //   // {
  //   //   eyes.setPixelColor(0, 155, 0, 0);
  //   //   fiberTail.setPixelColor(0, 0, 255, 0);
  //   //   fiberHead.setPixelColor(0, 0, 0, 255);
  //   //   eyes.setBrightness(50);
  //   //   fiberTail.setBrightness(p);
  //   //   fiberHead.setBrightness(p);

  //   //   // all three must be shown in this order - otherwise it will flicker
  //   //   fiberTail.show();
  //   //   fiberHead.show();
  //   //   eyes.show();
  //   // }

  //   // if (peak > lastPeak)
  //   // {
  //   //   lastPeak = peak;
  //   // }
  //   // // leftWingLinear(peak * 255.0, CHSV(255, 255, 255));
  //   // // rightWingLinear(peak * 255.0, CHSV(255, 255, 255));

  //   // lastPeak *= 0.99;
  // }
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

class NewAudio : public Effect
{
public:
  void setup()
  {

    currentDelay = 0;
    // shouldClear = true;
    shouldClear = false;
    shouldShow = true;
    potentiometerControlsBrightness = 0;
    usePixies = 1;

    // FastLED.setBrightness(255);

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

  // elapsedMicros fps0;

  void draw()
  {
    static int fb = 0;

    fadeToBlackBy(leds, NUM_LEDS, 3);

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
        // fiberTail.setPixelColor(0, c.r, c.g, c.b);
        // eyes.setPixelColor(0, c.r, c.g, c.b);
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

      // fiberTail.setBrightness(fb);
      // fiberTail.show();

      if (buttonState[EYE_STATE] == 1)
      {
        // eyes.setBrightness(fb);
        // eyes.show();
      }

      for (int i = 0; i < BEAT_BANDS; i += 1)
        if (count[i] == 0)
        {
          uint8_t h = beatsin8(15, 0, 255);
          int hue = beatsin8(10, 0, 255) + i * 4;
          // int hue = i * 5;

          int from = map(i, 0, BEAT_BANDS, 0, 255);
          int to = map(i + 1, 0, BEAT_BANDS, 0, 255);
          CRGB c = CHSV(hue, 230, 255);

          outer.lerpFromTo(max(from - 2, 0), min(to + 2, 255), c);
          coil.lerpFromTo(max(from - 2, 0), min(to + 2, 255), c);
          inner.lerpFromTo(max(from - 2, 0), min(to + 2, 255), c);

          // leftWingLinearLerpFromTo(from, to, c);
          // rightWingLinearLerpFromTo(from, to, c);

          // bodyBack.lerpFromTo(from, to, c);
          // bodyFront.lerpFromTo(from, to, c);

          // headLeft.lerpFromTo(from, to, c);
          // headRight.lerpFromTo(from, to, c);

          // fiberLeft.lerpFromTo(from, to, c);
          // fiberRight.lerpFromTo(from, to, c);

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
};

class Heartbeat : public Effect
{
public:
  elapsedMillis timeElapsed;
  void setup()
  {
    currentDelay = 10;
    // potentiometerControlsBrightness = 0;
    // usePixies = 1;
    shouldClear = false;
  }

  void draw()
  {

    // for (int i = 0; i < 7; i++)
    // {
    //   // fiberLeds[i] = CHSV(30 * i + beatsin8(10), 200, 255);
    //   // fiberLeds[i] = CRGB::Yellow; //CHSV(30 * i + beatsin8(10), 200, 255);
    //   strip.setPixelColor(i, to_rgbw(fiberLeds[i]));
    // }

    static int fadeSpeed = 5;
    coil.fade(fadeSpeed);
    fadeToBlackBy(fiberLeds, 7, fadeSpeed);
    fadeToBlackBy(fishLeds, NUM_FISH_LEDS, fadeSpeed);

    static int b = 0;

    if (timeElapsed > 1000)
    {
      timeElapsed = 0;
      fadeSpeed = 15;
      b = 1;
      coil.fill(CHSV(0, 255, 200));
      for (int i = 0; i < 7; i++)
      {
        fiberLeds[i] = CHSV(0, 255, 255);
      }
      for (int i = 0; i < NUM_FISH_LEDS; i++)
      {
        fishLeds[i] = CHSV(0, 255, 255);
      }
    }
    if (timeElapsed > 200 && b == 1)
    {
      fadeSpeed = 10;
      b = 0;
      coil.fill(CHSV(0, 255, 200));
      for (int i = 0; i < 7; i++)
      {
        fiberLeds[i] = CHSV(0, 255, 255);
      }
      for (int i = 0; i < NUM_FISH_LEDS; i++)
      {
        fishLeds[i] = CHSV(0, 255, 255);
      }
    }
  }
};

class FTest : public Effect
{
public:
  void setup()
  {
    // potentiometerControlsBrightness = 0;
    Serial.println("setup Ftest");
    FastLED.setDither(0);
    FastLED.setBrightness(255);
  }

  void draw()
  {

    Serial.println("draw");
    currentDelay = 0;
    static CRGB stripColor[5] = {CRGB::Green, CRGB::Red, CRGB::Blue, CRGB::Yellow, CRGB::White};

    static uint8_t z = 0;
    EVERY_N_MILLISECONDS(1000)
    {
      z++;
      z = z % 5;
      Serial.println(z);
    };

    CRGB c = stripColor[z];
    CRGB h = c.fadeLightBy(170);
    powerLeds[0] = h;
    powerLeds[1] = h;

    for (int i = 0; i < 7; i++)
    {

      fiberLeds[i] = c;
    }

    fill_solid(fishLeds, NUM_FISH_LEDS, CRGB::Black);
    for (int i = 0; i < NUM_FISH_LEDS; i++)
    // for (int i = 7; i < NUM_FISH_LEDS; i++)
    {
      fishLeds[i] = c;
    }

    fadeToBlackBy(fishLeds, NUM_FISH_LEDS, 200);

    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
      for (int j = 0; j < i + 1; j++)
      {
        outerSegments[i].at(j + 20, stripColor[z]);
        innerSegments[i].at(j + 20, stripColor[z]);
        coil.at(j, stripColor[z]);
      }
    }
    for (int i = 0; i < 10; i++)
    {
      coil.at(i, c);
    }
  }
};

class ColorWheelWithSparkels : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
    shouldClear = false;
  }
  void draw()
  {
    static elapsedMillis timer;
    if (timer < 20)
      return;
    timer = 0;

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
    baselineFishAndFibers();
  }
};

class Streaming : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
    shouldClear = false;
  }

  void draw()
  {
    static elapsedMillis timer;
    if (timer < beatsin8(30))
      return;
    timer = 0;
    CRGB c;
    static uint8_t idx = 0;
    if (idx > 2)
    {
      idx = 0;
      c = CHSV(beatsin8(10), 180, 200);
    }
    else
    {
      idx++;
      c = CRGB::Black;
    }
    *outerSegments[0][0] = c;

    for (int i = NUM_LEDS_PER_SEGMENT - 1; i > 0; i--)
    {
      CRGB p = *outerSegments[0][i - 1];
      for (int j = 0; j < NUM_SEGMENTS; j++)
      {

        *outerSegments[j][i] = p;
        *innerSegments[j][i] = p;
      }
    }
  }
};

class AccelTest : public Effect
{
public:
  void setup()
  {
    shouldClear = false;
  }

  void printEvent(sensors_event_t *event)
  {
    Serial.print(event->type);
    double x = -1000000, y = -1000000, z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER)
    {
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION)
    {
      x = event->orientation.x;
      y = event->orientation.y;
      z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
    {
      x = event->magnetic.x;
      y = event->magnetic.y;
      z = event->magnetic.z;
    }
    else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR))
    {
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
    }

    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
  }

  double xPos = 0, yPos = 0, headingVel = 0;
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

  //velocity = accel*dt (dt in seconds)
  //position = 0.5*accel*dt^2
  double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
  double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

  void draw()
  {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    fadeToBlackBy(leds, NUM_LEDS, 13);

    static uint32_t t = millis();

    if (millis() - t < BNO055_SAMPLERATE_DELAY_MS)
    {
      return;
    }

    sensors_event_t orientationData, linearAccelData, accelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

    // velocity of sensor in the direction it's facing
    headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

    int pp = orientationData.orientation.x / 360.0 * NUM_SEGMENTS;

    int to = linearAccelData.acceleration.y * 25;

    if (linearAccelData.acceleration.y > 6)
    {
      powerLeds[0] = CRGB(255, 255, 255);
      powerLeds[1] = CRGB(255, 255, 255);
    }
    else
    {
      powerLeds[0] = CRGB::Black;
      powerLeds[1] = CRGB::Black;
    }

    if (to < 3)
      to = 0;

    for (int i = 0; i < NUM_LEDS_PER_SEGMENT; i++)
    {

      *outerSegments[pp][i] = CHSV(i + beatsin8(6) * 2 + pp * 30, 255, 255);
      *outerSegments[(pp + 3) % 6][i] = CHSV(i + beatsin8(6) * 2 + pp * 30, 255, 255);
    }
    inner.lerpToReverse(max(0, to), CRGB::Blue);
    coil.lerpTo(max(0, to), CHSV(linearAccelData.acceleration.y * 30, 200, 200));
    // outer.lerpTo(max(0, linearAccelData.acceleration.y * 40), CRGB::Blue);

    // drehen ist x achse beschleuinigen
    baselineFishAndFibers();

    EVERY_N_MILLISECONDS(250)
    {

      Serial.print("Heading: ");
      Serial.println(orientationData.orientation.x);
      // Serial.print("Position: ");
      // Serial.print(xPos);
      // Serial.print(" , ");
      // Serial.println(yPos);
      // Serial.print("Speed: ");
      // Serial.println(headingVel);
      Serial.print("linearAccel");
      Serial.print(" x:");
      Serial.print(linearAccelData.acceleration.x);
      Serial.print(" y:");
      Serial.print(linearAccelData.acceleration.y);
      Serial.print(" z:");
      Serial.println(linearAccelData.acceleration.z);
      // Serial.println("accel");
      // this->printEvent(&accelData);
      // Serial.println("-------");

      // uint8_t system, gyro, accel, mag = 0;
      uint8_t sensorStatus[4] = {0, 0, 0, 0};
      bno.getCalibration(&sensorStatus[0], &sensorStatus[1], &sensorStatus[2], &sensorStatus[3]);

      static CRGB states[4] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green};

      // if (PAD_CONNECTED)
      // {
      //   for (int i = 0; i < 4; i++)
      //   {
      //     CRGB c = states[sensorStatus[i]];
      //     pad.pixels.setPixelColor(i, c.r, c.g, c.b);
      //   }
      //   pad.pixels.show();
      // }
    }
  }
};

class FiberAndFishsEffect : public Effect
{
public:
  void setup()
  {
    currentDelay = 0;
  }
  void draw()
  {
    // the fishes will pulse
    for (int i = NUM_FISH_LEDS - 1; i > NUM_FISH_LEDS - 6; i--)
    {
      fishLeds[i] = CHSV(0, 200, 80);
    }
    for (int i = 0; i < NUM_FISH_LEDS - 6; i++)
    {
      fishLeds[i] = CHSV(beatsin8(3) + i * 30, 250, map8(beatsin8(10), 5, 200));
    }
    fadeToBlackBy(fiberLeds, 7, 25);
    for (int i = 0; i < 7; i++)
    {
      fiberLeds[i] = CHSV(30 * i + beatsin8(10), 200, 255);
    }
  }
};

class JellyColorEffect : public Effect
{
  CRGBPalette16 paletteA;
  CRGBPalette16 paletteB;
  CRGBPalette16 palette;

public:
  void setup()
  {
    currentDelay = 0;

    fill_solid(palette, 16, CRGB::Black);
    paletteA[0] = CRGB::Navy;
    paletteA[1] = CRGB::CornflowerBlue;
    paletteA[4] = CRGB::Indigo;
    paletteA[5] = CRGB::MediumVioletRed;
    paletteA[6] = CRGB::BlueViolet;
    paletteA[9] = CRGB::DarkBlue;
    paletteA[10] = CRGB::Fuchsia;
    paletteA[12] = CRGB::LightSeaGreen;
    paletteA[13] = CRGB::DarkSlateBlue;

    paletteB = OceanColors_p;
  }
  void draw()
  {
    int b = beatsin8(2);

    uint8_t beat = beat8(30);
    uint8_t mixer = cubicwave8(beat);
    blend(paletteA, paletteB, palette, 16, mixer);

    for (int i = 0; i < NUM_LEDS_PER_SEGMENT; i++)
    {

      for (int8_t j = 0; j < NUM_SEGMENTS; j++)
      {

        innerSegments[j].at(i, ColorFromPalette(palette, beatsin8(7) + i * 2, b, LINEARBLEND));
        outerSegments[j].at(NUM_LEDS_PER_SEGMENT - 1 - i, ColorFromPalette(palette, beatsin8(7) + i * 2, b, LINEARBLEND));
        // innerSegments[j].at(i, ColorFromPalette(palette, idx, 200, LINEARBLEND));
      }
    }
    for (int i = 0; i < NUM_COIL_LEDS; i++)
    {
      coil.at(i, ColorFromPalette(palette, beat8(10) + i * 4, 255 - b, LINEARBLEND));
    }
    for (int i = 0; i < NUM_FIBERS; i++)
    {
      fishLeds[i] = ColorFromPalette(palette, beat8(4) + i * 20, 255, LINEARBLEND);
    }
    for (int i = 0; i < NUM_FIBERS; i++)
    {
      fiberLeds[i] = ColorFromPalette(palette, beat8(4) + i * 20, 255, LINEARBLEND);
    }
  }
};

class Runner
{
public:
  int pos;
  int dir;
  int segment;
  Runner()
  {
    dir = random8(2);
    if (!dir)
      dir = -1;
    pos = random8(NUM_LEDS_PER_SEGMENT);
    segment = random8(6);
  }

  void advance()
  {
    pos += dir;
    if (pos >= 59)
    {
      pos = 59;
      dir = -1;
      segment = (segment + 3) % 6;
    }
    if (pos <= 0)
    {
      pos = 0;
      dir = 1;
      segment = abs((segment - 3) % 6);
    }
    // Serial.print("pos:");
    // Serial.print(pos);
    // Serial.print(" dir:");
    // Serial.print(dir);
    // Serial.print(" seg:");
    // Serial.println(segment);
  }
};

#define NUM_RUNNERS 12

class RunnersEffect : public Effect
{
public:
  Runner runners[NUM_RUNNERS];
  void setup()
  {
    currentDelay = 0;
    shouldClear = false;
  }
  void draw()
  {
    fadeToBlackBy(leds, NUM_LEDS, 70);
    static int dothue = 0;
    for (int i = 0; i < NUM_RUNNERS; i++)
    {
      runners[i].advance();
      *outerSegments[runners[i].segment][runners[i].pos] |= CHSV(dothue, 200, 255);
      *innerSegments[runners[i].segment][runners[i].pos] |= CHSV(dothue, 200, 255);
      dothue += 1;
    }
    fadeToBlackBy(coil.ledP(), coil.size, 10);

    EVERY_N_MILLISECONDS(200)
    {
      for (int i = 0; i < coil.size; i++)
      {
        *coil[i] |= CHSV(i * 2 + beatsin8(10), 230, 220);
      }
    }
  }
};

// the setup routine runs once when you press reset:
void setup()
{

  // add all effects
  effects = {
      // new FTest(),
      new FiberAndFishsEffect(),    // 0
      new Fibercrazy(),             // 1
      new Heartbeat(),              // 2
      new AccelTest(),              // 3
      new NewAudio(),               // 4
      new JellyAudio(),             // 5
      new JellyColorEffect(),       // 6
      new RunnersEffect(),          // 7
      new ColorWheelNew(),          // 8
      new ColorWheelWithSparkels(), // 9
      new Streaming(),              // 10
      new Rings(),                  // 11
      new Juggle(),                 // 12
      // new DropEffect(),
      new Bouncy(),            // 13
      new PrideEffect(),       // 14
      new IceSparkEffect(),    // 15
      new SinelonEffect(),     // 16
      new PixelFiringEffect(), // 17
      // new BlurEffect(),
      new PaletteTestEffect()}; // 18

  while (!Serial && (millis() <= 2000))
    ; // Wait for Serial interface
  Serial.begin(115200);

  Serial.println("boot jelly");

  // init 3w leds serial connection
  Serial.println("3W LED init");
  Serial1.setTX(26);
  Serial1.begin(115200);
  FastLED.addLeds<PIXIE, 26>(powerLeds, 2);

  Serial.println("LED init");
  fiberLedsNeoPixel.begin();

  // 2,14,7,8,6,20,21,5
  FastLED.addLeds<WS2811_PORTD, NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2811, 20, GRB>(fishLeds, NUM_FISH_LEDS);

  FastLED.setMaxPowerInVoltsAndMilliamps(5, 5000);
  FastLED.setBrightness(MAX_BRIGHTNESS);

  // clear();
  // FastLED.clear();
  // FastLED.show();

  // init radio!
  pinMode(REMOTE_RECEIVER_PIN, INPUT);
  remoteSwitch.enableReceive(REMOTE_RECEIVER_PIN);

  if (PAD_CONNECTED)
  {
    Serial.println("pad init");
    // init button pads and flash them once
    if (!pad.begin())
    {
      Serial.println("pad init failed!");
    }
    else
    {
      for (uint16_t i = 0; i < pad.pixels.numPixels(); i++)
      {

        pad.activateKey(i, SEESAW_KEYPAD_EDGE_RISING);
        pad.activateKey(i, SEESAW_KEYPAD_EDGE_FALLING);
        pad.registerCallback(i, readButton);
      }
      for (uint16_t i = 0; i < pad.pixels.numPixels(); i++)
      {
        CRGB c = CHSV(i * 20, 240, 240);
        pad.pixels.setPixelColor(i, c.r, c.g, c.b);
        pad.pixels.show();
        delay(25);
      }
      for (uint16_t i = 0; i < pad.pixels.numPixels(); i++)
      {
        pad.pixels.setPixelColor(i, 0x000000);
        pad.pixels.show();
        delay(25);
      }
    }
  }
  else
  {
    Serial.println("pad disabled!");
  }

  Serial.println("motion init");

  // Initialise the motion sensor
  if (!bno.begin())
  {
    Serial.print("motion sensor error");
  }
  else
  {
    Serial.println("motion sensor OK");
    delay(200);
    int8_t temp = bno.getTemp();
    Serial.print("temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    bno.setExtCrystalUse(true);
  }

  AudioMemory(8);

  pinMode(TEENSY_LED, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);

  adc->setAveraging(16, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1);
  // adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0);
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1); // doesn't work but we hacked the audio library and core to still  work
  adc->adc1->recalibrate();

  delay(10);

  // start with mode number 0
  // modes[0][1]();
  effects[0]->setup();
}

void loop()
{
  runScheduluer();

  if (shouldClear)
    clear();

  effects[currentMode]->draw();

  if (shouldShow)
  {
    if (useFibers)
    {
      for (int i = 0; i < 7; i++)
      {
        fiberLedsNeoPixel.setPixelColor(i, to_rgbw(fiberLeds[i]));
      }
    }
    fiberLedsNeoPixel.show();
    FastLED.show();
  }
  fps++;

  if (scheduleStrobo)
  {
    scheduleStrobo--;
    if (scheduleStrobo < 0)
      scheduleStrobo = 0;
    clear();
    int b = FastLED.getBrightness();
    FastLED.setBrightness(255);

    powerLeds[0] = CRGB::White;
    powerLeds[1] = CRGB::White;
    FastLED.show();
    delay(30);
    powerLeds[0] = CRGB::Black;
    powerLeds[1] = CRGB::Black;
    FastLED.show();

    FastLED.setBrightness(b);
  }

  if (PAD_CONNECTED)
  {
    EVERY_N_MILLISECONDS(100) { checkButtons(); }
  }
  EVERY_N_MILLISECONDS(250) { checkPotentiometer(); }
  EVERY_N_MILLISECONDS(50) { readRemoteSwitch(); }
  EVERY_N_MILLISECONDS(1000) { testled(); }
  EVERY_N_MILLISECONDS(100) { checkSerial(); }
  EVERY_N_MILLISECONDS(1000) { showFps(); }

  if (currentDelay > 0)
    delay(currentDelay);
}
