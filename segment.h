class Segment
{
public:
  int start, end, size;
  Segment(int s, int e)
  {
    start = s;
    end = e;
    size = e - s;
  }
  void fill(CRGB c)
  {
    for (int i = start; i < end; i++)
    {
      leds[i] = c;
    }
  }
  uint16_t lerp(uint8_t x)
  {
    return lerp16by8(start, end, x);
  }
  void lerpTo(uint8_t x, CRGB c)
  {
    for (int i = start; i < lerp(x); i++)
    {
      leds[i] = c;
    }
  }

  void lerpFromTo(uint8_t x, uint8_t y, CRGB c)
  {
    uint8_t a = min(x, y);
    uint8_t b = max(x, y);
    for (int i = lerp(a); i < lerp(b); i++)
    {
      leds[i] = c;
    }
  }

  void lerpFromToReverse(uint8_t x, uint8_t y, CRGB c)
  {
    for (int i = lerp(255 - x); i > lerp(255 - y); i--)
    {
      leds[i] = c;
    }
  }

  void lerpToReverse(uint8_t x, CRGB c)
  {
    for (int i = end; i > lerp(255 - x); i--)
    {
      leds[i] = c;
    }
  }
  void lerpAt(uint8_t x, CRGB c)
  {
    leds[lerp(x)] = c;
  }
  void lerpAtAdd(uint8_t x, CRGB c)
  {
    leds[lerp(x)] += c;
  }
};

#define LW_FRONT_START 0
#define LW_FRONT_PEAK 28
#define LW_FRONT_END 46

#define LW_MIDDLE_START 0 + NUM_LEDS_PER_STRIP
#define LW_MIDDLE_PEAK 33 + NUM_LEDS_PER_STRIP
#define LW_MIDDLE_END 68 + NUM_LEDS_PER_STRIP

#define LW_BACK_START 46
#define LW_BACK_PEAK 73
#define LW_BACK_END 101

#define RW_FRONT_START 0 + (NUM_LEDS_PER_STRIP * 2)
#define RW_FRONT_PEAK 28 + (NUM_LEDS_PER_STRIP * 2)
#define RW_FRONT_END 45 + (NUM_LEDS_PER_STRIP * 2)

#define RW_MIDDLE_START 0 + (NUM_LEDS_PER_STRIP * 3)
#define RW_MIDDLE_PEAK 31 + (NUM_LEDS_PER_STRIP * 3)
#define RW_MIDDLE_END 68 + (NUM_LEDS_PER_STRIP * 3)

#define RW_BACK_START 45 + (NUM_LEDS_PER_STRIP * 2)
#define RW_BACK_PEAK 70 + (NUM_LEDS_PER_STRIP * 2)
#define RW_BACK_END 96 + (NUM_LEDS_PER_STRIP * 2)

#define BODY_START 0 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_END 50 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_AND_HEAD_END 70 + (NUM_LEDS_PER_STRIP * 4)

#define BODY_FRONT_START 36 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_FRONT_END 50 + (NUM_LEDS_PER_STRIP * 4)

#define BODY_BACK_START 0 + (NUM_LEDS_PER_STRIP * 4)
#define BODY_BACK_END 36 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_START 50 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_END 70 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_LEFT_START 50 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_LEFT_END 60 + (NUM_LEDS_PER_STRIP * 4)

#define HEAD_RIGHT_START 60 + (NUM_LEDS_PER_STRIP * 4)
#define HEAD_RIGHT_END 70 + (NUM_LEDS_PER_STRIP * 4)

Segment lwFront = Segment(LW_FRONT_START, LW_FRONT_END);
Segment lwFrontInner = Segment(LW_FRONT_START, LW_FRONT_PEAK);
Segment lwFrontOuter = Segment(LW_FRONT_PEAK, LW_FRONT_END);

Segment lwMiddleTop = Segment(LW_MIDDLE_START, LW_MIDDLE_PEAK);
Segment lwMiddleBottom = Segment(LW_MIDDLE_PEAK, LW_MIDDLE_END);

Segment lwBackTop = Segment(LW_BACK_START, LW_BACK_PEAK);
Segment lwBackBottom = Segment(LW_BACK_PEAK, LW_BACK_END);

Segment rwFront = Segment(RW_FRONT_START, RW_FRONT_END);
Segment rwFrontInner = Segment(RW_FRONT_START, RW_FRONT_PEAK);
Segment rwFrontOuter = Segment(RW_FRONT_PEAK, RW_FRONT_END);
Segment rwMiddleTop = Segment(RW_MIDDLE_PEAK, RW_MIDDLE_END);
Segment rwMiddleBottom = Segment(RW_MIDDLE_START, RW_MIDDLE_PEAK);
Segment rwBackTop = Segment(RW_BACK_START, RW_BACK_PEAK);
Segment rwBackBottom = Segment(RW_BACK_PEAK, RW_BACK_END);

Segment bodyFront = Segment(BODY_FRONT_START, BODY_FRONT_END);
Segment bodyBack = Segment(BODY_BACK_START, BODY_BACK_END);

Segment headLeft = Segment(HEAD_LEFT_START, HEAD_LEFT_END);
Segment headRight = Segment(HEAD_RIGHT_START, HEAD_RIGHT_END);

Segment fiberLeft = Segment(NUM_LEDS_PER_STRIP * 2 + 96, NUM_LEDS_PER_STRIP * 2 + 99);
Segment fiberRight = Segment(101, 104);

#define LW_LERP_OFFSET 100.0
#define RW_LERP_OFFSET 100.0

void leftWingLinearLerp(uint8_t x, CRGB c)
{
  lwFrontInner.lerpAt(x, c);
  lwMiddleTop.lerpAt(x, c);
  lwMiddleBottom.lerpAt(255 - x, c);
  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    lwBackBottom.lerpAt(255 - xx, c);
    lwBackTop.lerpAt(xx, c);
  }
}
void rightWingLinearLerp(uint8_t x, CRGB c)
{
  rwFrontInner.lerpAt(x, c);
  rwMiddleTop.lerpAt(255 - x, c);
  rwMiddleBottom.lerpAt(x, c);
  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    rwBackBottom.lerpAt(255 - xx, c);
    rwBackTop.lerpAt(xx, c);
  }
}
void wingLinearLerp(uint8_t x, CRGB c)
{
  rightWingLinearLerp(x, c);
  leftWingLinearLerp(x, c);
}

void leftWingLinearLerpTo(uint8_t x, CRGB c)
{
  lwFrontInner.lerpTo(x, c);
  lwMiddleTop.lerpTo(x, c);
  lwMiddleBottom.lerpToReverse(x, c);
  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    lwBackBottom.lerpTo(255 - xx, c);
    lwBackTop.lerpTo(xx, c);
  }
}
void rightWingLinearLerpTo(uint8_t x, CRGB c)
{
  rwFrontInner.lerpTo(x, c);
  rwMiddleTop.lerpToReverse(x, c);
  rwMiddleBottom.lerpTo(x, c);
  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    rwBackBottom.lerpTo(255 - xx, c);
    rwBackTop.lerpTo(xx, c);
  }
}
void wingLinearLerpTo(uint8_t x, CRGB c)
{
  rightWingLinearLerpTo(x, c);
  leftWingLinearLerpTo(x, c);
}

void leftWingLinearLerpToFull(uint8_t x, CRGB c)
{
  lwFront.lerpTo(x, c);
  lwMiddleTop.lerpTo(x, c);
  lwMiddleBottom.lerpToReverse(x, c);
  if (float(x) > LW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
    lwBackBottom.lerpTo(255 - xx, c);
    lwBackTop.lerpTo(xx, c);
  }
}
void rightWingLinearLerpToFull(uint8_t x, CRGB c)
{
  rwFront.lerpTo(x, c);
  rwMiddleTop.lerpToReverse(x, c);
  rwMiddleBottom.lerpTo(x, c);
  if (float(x) > RW_LERP_OFFSET)
  {
    uint8_t xx = uint8_t((float(x) - RW_LERP_OFFSET) * (255.0 / (255.0 - RW_LERP_OFFSET)));
    rwBackBottom.lerpTo(255 - xx, c);
    rwBackTop.lerpTo(xx, c);
  }
}
void wingLinearLerpToFull(uint8_t x, CRGB c)
{
  rightWingLinearLerpToFull(x, c);
  leftWingLinearLerpToFull(x, c);
}

void leftWingLinearLerpFromTo(uint8_t from, uint8_t to, CRGB c)
{
  lwFront.lerpFromTo(from, to, c);
  lwMiddleTop.lerpFromTo(from, to, c);
  lwMiddleBottom.lerpFromToReverse(from, to, c);

  lwBackTop.lerpFromTo(from, to, c);
  lwBackBottom.lerpFromToReverse(from, to, c);

  // if (float(x) > LW_LERP_OFFSET)
  // {
  //   uint8_t xx = uint8_t((float(x) - LW_LERP_OFFSET) * (255.0 / (255.0 - LW_LERP_OFFSET)));
  //   lwBackBottom.lerpTo(255 - xx, c);
  //   lwBackTop.lerpTo(xx, c);
  // }
}
void rightWingLinearLerpFromTo(uint8_t from, uint8_t to, CRGB c)
{
  rwFront.lerpFromTo(from, to, c);
  rwMiddleTop.lerpFromToReverse(from, to, c);
  rwMiddleBottom.lerpFromTo(from, to, c);
  rwBackTop.lerpFromTo(from, to, c);
  rwBackBottom.lerpFromToReverse(from, to, c);
}
