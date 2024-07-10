void clear()
{
  memset(leds, 0, sizeof(leds));
  memset(fiberLeds, 0, sizeof(fiberLeds));
  memset(fishLeds, 0, sizeof(fishLeds));
}

void testled()
{
  static uint8_t lvl = HIGH;
  digitalWrite(TEENSY_LED, lvl);
  if (lvl == HIGH)
    lvl = LOW;
  else
    lvl = HIGH;
}

void setMode(uint8_t mode)
{
  Serial.print("SETTING MODE: ");
  Serial.println(mode);

  previousMode = currentMode;
  currentMode = mode;
  if (currentMode < 0)
    currentMode = effects.size() - 1;
  // setBrightness(MAX_BRIGHTNESS);

  potentiometerControlsBrightness = 1;
  currentDelay = 0;
  shouldClear = 1;
  shouldShow = 1;

  clear();
  for (int i = 0; i < NUM_FIBERS; i++)
  {
    fiberLedsNeoPixel.setPixelColor(i, 0);
  }
  FastLED.show();

  effects[currentMode]->setup();
}

int8_t getNextMode(int8_t dir)
{
  int8_t newMode;
  newMode = currentMode + dir;

  if (newMode < 0)
  {
    newMode = effects.size() - 1;
  }
  else if (newMode >= effects.size())
  {
    newMode = 0;
  }
  return newMode;
}

void nextMode(int8_t dir)
{
  int8_t newMode = getNextMode(dir);
  setMode(newMode);
}

int amax(int *array, int size)
{
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i = 1; i < size; i++)
  {
    if (max < array[i])
    {
      max = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}
int amode(int *a, int size)
{
  int modeMap[size];
  int maxEl = a[0];
  int maxCount = 1;

  for (int i = 0; i < size; i++)
  {
    int el = a[i];
    if (modeMap[el] == 0)
    {
      modeMap[el] = 1;
    }
    else
    {
      modeMap[el]++;
    }

    if (modeMap[el] > maxCount)
    {
      maxEl = el;
      maxCount = modeMap[el];
    }
  }
  return maxEl;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t to_rgbw(CRGB c)
{

  float tM = max(c.r, max(c.g, c.b));

  if (tM == 0)
    return 0;

  //This section serves to figure out what the color with 100% hue is
  float multiplier = 255.0f / tM;
  float hR = c.r * multiplier;
  float hG = c.g * multiplier;
  float hB = c.b * multiplier;

  //This calculates the Whiteness (not strictly speaking Luminance) of the color
  float M = max(hR, max(hG, hB));
  float m = min(hR, min(hG, hB));
  float Luminance = ((M + m) / 2.0f - 127.5f) * (255.0f / 127.5f) / multiplier;

  //Calculate the output values
  int Wo = int(Luminance);
  int Bo = int(c.b - Luminance);
  int Ro = int(c.r - Luminance);
  int Go = int(c.g - Luminance);

  //Trim them so that they are all between 0 and 255
  if (Wo < 0)
    Wo = 0;
  if (Bo < 0)
    Bo = 0;
  if (Ro < 0)
    Ro = 0;
  if (Go < 0)
    Go = 0;
  if (Wo > 255)
    Wo = 255;
  if (Bo > 255)
    Bo = 255;
  if (Ro > 255)
    Ro = 255;
  if (Go > 255)
    Go = 255;

  return Bo + (Go << 8) + (Ro << 16) + (Wo << 24);
}
