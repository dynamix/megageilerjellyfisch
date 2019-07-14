void clear()
{
  memset(leds, 0, sizeof(leds));
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
  previousMode = currentMode;
  currentMode = mode;
  if (currentMode < 0)
    currentMode = ARRAY_SIZE(modes) - 1;
  // setBrightness(MAX_BRIGHTNESS);
  usePotentiometer = 1;
  currentDelay = 0;
  shouldClear = 1;
  shouldShow = 1;
  usePixies = 0;
  overridePixieColor = 0;
  modes[currentMode][1]();
}

void nextMode(int8_t dir)
{
  int8_t newMode;
  newMode = currentMode + dir;

  if (newMode < 0)
  {
    newMode = ARRAY_SIZE(modes) - 1;
  }
  else if (newMode >= ARRAY_SIZE(modes))
  {
    newMode = 0;
  }
  Serial.print("NEXT MODE");
  Serial.println(newMode);
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
