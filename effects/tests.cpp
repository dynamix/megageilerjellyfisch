void bodyTest()
{
  static int pos = 0;
  // static uint8_t hue = 0;

  // FastLED.setBrightness(75);
  // hue++;
  clear();

  // EVERY_N_MILLISECONDS(10)
  // {
  pos++;
  // }
  pos %= NUM_LEDS_PER_STRIP;

  // pos %= 10;

  // EVERY_N_MILLISECONDS(500) { }

  // for (int i = BODY_FRONT_START; i < BODY_FRONT_END; i++)
  // {
  //   leds[i] = CRGB(i * 10, 0, 255);
  // }
  // for (int i = BODY_BACK_START; i < BODY_BACK_END; i++)
  // {
  //   leds[i] = CRGB(255, 0, i * 10);
  // }

  // for (int i = 0; i < NUM_STRIPS; i++)
  // {
  // leds[(i * NUM_LEDS_PER_STRIP) + pos] = CRGB(255, 0, 0);
  for (int j = 0; j < pos; j++)
  // for (int j = 0; j < NUM_LEDS_PEdR_STRIP; j++)
  // for (int j = 0; j < 20; j++)
  {
    for (int i = 0; i < NUM_STRIPS; i++)
    {
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB(0, 0, 255);
      // leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV(hue + j, 255, 255);
      // }
      // }

      // FastLED.show();
      // delay(25);
    }
  }
  // LEDS.show();
  FastLED.show();
  // FastLED.delay(10);
  delay(10);
}

void basicEyeTest()
{
  static uint8_t b = 0;
  EVERY_N_MILLISECONDS(500)
  {
    if (b == 0)
      b = 50;
    else
      b = 0;
  }
  eyes.setBrightness(b);
  eyes.setPixelColor(0, 255, 0, 0);
  // eyes.setPixelColor(0, 155, 45, 25);
  eyes.show();
}

void basicFiberHeadTest()
{
  static uint8_t b = 0;
  EVERY_N_MILLISECONDS(500)
  {
    if (b == 0)
      b = 255;
    else
      b = 0;
  }
  fiberHead.setBrightness(b);
  // fiberHead.setBrightness(255);
  fiberHead.setPixelColor(0, 255, 255, 255);
  // fiberHead.setPixelColor(0, 255, 255, 255);
  fiberHead.show();
}

void basicFiberTailTest()
{
  static uint8_t b = 255;
  // EVERY_N_MILLISECONDS(500)
  // {
  //   if (b == 0)
  //     b = 255;
  //   else
  //     b = 0;
  // }
  fiberTail.setBrightness(b);
  fiberTail.setPixelColor(0, 0, 255, 0);
  fiberTail.show();
}
