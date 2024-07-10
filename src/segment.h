
class Segment
{
private:
  inline int pos(int p)
  {
    if (reverse)
      return size - p - 1;
    return p;
  }

public:
  int start, end, size;
  bool reverse;
  CRGB *_leds;
  Segment(int s, int e, bool reverse = false, CRGB *_leds = leds)
  {
    start = s;
    end = e;
    size = e - s;
    this->reverse = reverse;
    this->_leds = _leds;
  }
  CRGB *ledP()
  {
    return &_leds[start];
  }
  CRGB *operator[](int idx)
  {
    return &_leds[pos(idx) + start];
  }
  void atAdd(uint8_t idx, CRGB c)
  {
    _leds[pos(idx) + start] += c;
  }
  void at(int idx, CRGB c)
  {
    _leds[pos(idx) + start] = c;
  }
  void fill(CRGB c)
  {
    for (int i = start; i < end; i++)
    {
      _leds[i] = c;
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
      if (reverse)
        _leds[end - (i - start)] = c;
      else
        _leds[pos(i)] = c;
    }
  }

  void lerpFromTo(uint8_t x, uint8_t y, CRGB c)
  {
    uint8_t a = min(x, y);
    uint8_t b = max(x, y);
    int _to = lerp(b);
    int _from = lerp(a);
    for (int i = _from; i < _to; i++)
    {
      // Serial.print("start=");
      // Serial.print(start);
      // Serial.print(" end=");
      // Serial.print(end);
      // Serial.print(" x=");
      // Serial.print(x);
      // Serial.print(" y=");
      // Serial.print(y);

      // Serial.print(" a=");
      // Serial.print(a);
      // Serial.print(" lerp_a=");
      // Serial.print(lerp(a));
      // Serial.print(" lerp_b=");
      // Serial.print(lerp(b));
      // Serial.print(" b=");
      // Serial.print(b);
      // Serial.print(" i=");
      // Serial.print(i);
      if (reverse)
      {
        // Serial.print(" D=");
        // Serial.print(end - (i - start) - 1);

        // _leds[i] = c;
        _leds[end - (i - start)] = c;
      }
      else
        _leds[i] = c;
      // Serial.print(" : ");
      // Serial.println("");
    }
  }

  void
  lerpFromToReverse(uint8_t x, uint8_t y, CRGB c)
  {
    for (int i = lerp(255 - x); i > lerp(255 - y); i--)
    {
      _leds[pos(i)] = c;
    }
  }

  void lerpToReverse(uint8_t x, CRGB c)
  {
    for (int i = end; i > lerp(255 - x); i--)
    {
      if (reverse)
        _leds[end - (i - start)] = c;
      else
        _leds[i] = c;
    }
  }
  void lerpAt(uint8_t x, CRGB c)
  {
    _leds[pos(lerp(x))] = c;
  }
  void lerpAtAdd(uint8_t x, CRGB c)
  {
    _leds[pos(lerp(x))] += c;
  }
  void fade(uint8_t v)
  {
    ::fadeToBlackBy(&leds[start], size, v);
  }
};

class SegmentGroup
{

public:
  Segment *_segments;
  int _n;

  SegmentGroup(Segment *segments, int n)
  {
    _segments = segments;
    _n = n;
  }
  void at(int idx, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].at(idx, c);
    }
  }
  void atAdd(int idx, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].atAdd(idx, c);
    }
  }
  void fill(CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].fill(c);
    }
  }
  void lerpTo(uint8_t x, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpTo(x, c);
    }
  }

  void lerpFromTo(uint8_t x, uint8_t y, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpFromTo(x, y, c);
    }
  }

  void lerpFromToReverse(uint8_t x, uint8_t y, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpFromToReverse(x, y, c);
    }
  }

  void lerpToReverse(uint8_t x, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpToReverse(x, c);
    }
  }
  void lerpAt(uint8_t x, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpAt(x, c);
    }
  }
  void lerpAtAdd(uint8_t x, CRGB c)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].lerpAtAdd(x, c);
    }
  }
  void fade(uint8_t v)
  {
    for (int i = 0; i < _n; i++)
    {
      _segments[i].fade(v);
    }
  }
};

#define PIXELS_PER_SEGMENT 60

Segment outerSegments[6] = {
    Segment(0, PIXELS_PER_SEGMENT),
    Segment(PIXELS_PER_SEGMENT * 2, PIXELS_PER_SEGMENT * 3),
    Segment(PIXELS_PER_SEGMENT * 4, PIXELS_PER_SEGMENT * 5),
    Segment(PIXELS_PER_SEGMENT * 6, PIXELS_PER_SEGMENT * 7),
    Segment(PIXELS_PER_SEGMENT * 10, PIXELS_PER_SEGMENT * 11),
    Segment(PIXELS_PER_SEGMENT * 8, PIXELS_PER_SEGMENT * 9),
};

Segment innerSegments[6] = {
    Segment(PIXELS_PER_SEGMENT, PIXELS_PER_SEGMENT * 2, true),
    Segment(PIXELS_PER_SEGMENT * 3, PIXELS_PER_SEGMENT * 4, true),
    Segment(PIXELS_PER_SEGMENT * 5, PIXELS_PER_SEGMENT * 6, true),
    Segment(PIXELS_PER_SEGMENT * 7, PIXELS_PER_SEGMENT * 8, true),
    Segment(PIXELS_PER_SEGMENT * 11, PIXELS_PER_SEGMENT * 12, true),
    Segment(PIXELS_PER_SEGMENT * 9, PIXELS_PER_SEGMENT * 10, true),
};

#define COIL_START PIXELS_PER_SEGMENT * 12
#define NUM_COIL_RINGS 11
#define NUM_LEDS_PER_COIL_RING 7
#define NUM_COIL_LEDS 77

Segment coil = Segment(COIL_START, COIL_START + NUM_COIL_LEDS);

Segment fibers = Segment(0, NUM_FIBERS, false, fiberLeds);

Segment allSegments[12] = {
    Segment(0, PIXELS_PER_SEGMENT),
    Segment(PIXELS_PER_SEGMENT * 2, PIXELS_PER_SEGMENT * 3),
    Segment(PIXELS_PER_SEGMENT * 4, PIXELS_PER_SEGMENT * 5),
    Segment(PIXELS_PER_SEGMENT * 6, PIXELS_PER_SEGMENT * 7),
    Segment(PIXELS_PER_SEGMENT * 8, PIXELS_PER_SEGMENT * 9),
    Segment(PIXELS_PER_SEGMENT * 10, PIXELS_PER_SEGMENT * 11),
    // inner
    Segment(PIXELS_PER_SEGMENT, PIXELS_PER_SEGMENT * 2, true),
    Segment(PIXELS_PER_SEGMENT * 3, PIXELS_PER_SEGMENT * 4, true),
    Segment(PIXELS_PER_SEGMENT * 5, PIXELS_PER_SEGMENT * 6, true),
    Segment(PIXELS_PER_SEGMENT * 7, PIXELS_PER_SEGMENT * 8, true),
    Segment(PIXELS_PER_SEGMENT * 9, PIXELS_PER_SEGMENT * 10, true),
    Segment(PIXELS_PER_SEGMENT * 11, PIXELS_PER_SEGMENT * 12, true),
};

SegmentGroup outer(outerSegments, 6);
SegmentGroup inner(innerSegments, 6);
SegmentGroup all(allSegments, 12);
