             _/|       |\_
            /  |       |  \
           |    \     /    |
           |  \ /     \ /  |
           | \  |     |  / |
           | \ _\_/^\_/_ / |
           |    --\//--    |
            \_  \     /  _/
              \__  |  __/
                 \ _ /
                _/   \_   Project Phoenix (aka Hugo) ;-)
               / _/|\_ \  
                /  |  \   
                 / v \


# megageilervogel

A bird turned dragon (in its second year).

## Prerequisites 

For the current version you need.

- [Arduino 1.8.5](https://www.arduino.cc/en/Main/Software?)
- [latest Teensyduino](https://www.pjrc.com/teensy/td_download.html)
- a Teensy 3.6
- a RaspperyPi Zero W (for over the air updates)

## Setup

### Add the Teensy to know Arduino boards 
Add the following to your boards.txt (on OSX in /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/boards.txt)

```
teensy36.build.fcpu=180000000
teensy36.build.usbtype=USB_SERIAL
teensy36.build.keylayout=US_ENGLISH
teensy36.build.flags.optimize=-Os
teensy36.build.flags.ldspecs=--specs=nano.specs
```

### Needed libraries

A couple of libraries are needed to build the project. These need to be in your Arduino libraries folder. Usually `~/Documents/Arduino/libraries/` on macOS.

* [FastLed](https://github.com/FastLED/FastLED) to control the LEDs
* [Pixie](https://github.com/adafruit/Adafruit_Pixie) for the 3W LEDs
* [Trellis](https://github.com/adafruit/Adafruit_Trellis_Library) for the button pad
* [LSM9DS0](https://github.com/adafruit/Adafruit_LSM9DS0_Library) for the motion sensor
* [Adafruit Sensors](https://github.com/adafruit/Adafruit_Sensor) sensor library  
* [ADC](https://github.com/pedvide/ADC) to be able to use ADC1 independently of ADC0 which is in use by the Teensy Audio library

**Important** for the potentiometer to work correctly, the Teensy Audio library needed to be edited. Open `input_adc.cpp` (to be found in the Teensy audio library folder) and find the call to `analogReference(INTERNAL);` and comment it out. (See https://github.com/PaulStoffregen/Audio/blob/master/input_adc.cpp#L51)

# Remark

The code base is in no way idiomatic C/C++ (i.e. directly including files without headers) or written to please the eye. :-)

# Build

```
make
```

# Deploy

Over the air deployment relies on:
- the PI is connected to the local network
- the PI and the Teensy are correctly connected
- the teensyloader is available on the PI

```
make deploy
```

# Serial port on Pi

TODO
