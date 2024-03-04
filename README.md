## Espresso Machine PID controller using ESP32 and RainMaker

### Brief description

This project implements a PID/P controller to effectively control the temperature of an Espresso machine boiler, for a stable setpoint.
Another feature implemented is the control of the water pump and pre-infusion settings. Combined, these functionalities allow for
improved espresso extraction and consistency.

March 2024  
https://github.com/raffarost/espresso

Raffael Rostagno  
raffael.rostagno@gmail.com

### Software

#### Build and Flash firmware

Follow the ESP RainMaker Documentation [Get Started](https://rainmaker.espressif.com/docs/get-started.html) section to build and flash this firmware. Both RainMaker and ESP-IDF environments are necessary.

#### RainMaker

For the control interface, download [RainMaker](https://play.google.com/store/apps/details?id=com.espressif.rainmaker&hl=en&gl=US) on
your mobile, attach a USB cable to the serial port of the ESP32 board and do the wifi provisioning, after flashing the device with
the application firmware.

### Hardware

#### Components used in this project:

- ESP32 DevKitC v4 board
- Dimmer control circuit with Triac and zero-crossing signal
- Thermocouple type K with MAX6675 module
- Simple relay board (3.3V or 5V) with 3A or 5A current rating, 220V or 110V voltage rating
- ULN2003 or similar with free-wheeling diode, to drive relay
- Rapid prototype PCB for water pump drive assembly
- AC to 5V power supply to power *Embedded* devices
- Jumper wiring

#### Pinout and wiring

- MAX6675/Thermocouple

  | Signal   |   GPIO  |
  |----------|---------|
  | MISO     |    19   |
  | MOSI     |    NC   |
  | CLK      |    18   |
  | CS       |    25   |

- AC Dimmer (Heating element control)

  |   Signal         |   GPIO  |
  |------------------|---------|
  |ZC (Zero-cross)   |    5    |  
  |DIM (Dimmer)      |    33   |

- ULN2003/Relay (water pump control)

  |Signal     |   GPIO   |
  |-----------|----------|
  |Relay in   |    4     |

### Settings

#### Pinout

Pinout can be chosen according to personal needs and model of SoC used (SPI pins are fixed in some models).

#### Power grid frequency

For optimal control of the dimmer circuit, please adjust the following define according to your local power grid frequency:

```
#define GRID_FREQ       60
```

#### Type of controller

Three types of controlling algorithm for the heating element are provided:

```
#define PID         0
#define LOOKUP      1
#define PID_LOOKUP  2
```

The chosen control mechanism can be selected using the define below:

```
#define CONTROL_TYPE    LOOKUP
```

#### PID calibration

To calibrate the PID controller, the following symbols can be optimized for each application:

```
#define PID_KP  4.0f
#define PID_KI  2.0f
#define PID_KD  1.0f

#define PID_TAU 0.02f
```

#### P calibration (lookup)

To calibrate the P controller, the following vectors can be changed:

```
static float   deltaBkp[BKP_NUM] = {-10,  0, 0.5,  1,  2,  4, 10,  25,  50, 100};
static float controlSet[BKP_NUM] = {  0,  0,   1,  1,  1,  1,  1,  80, 100, 100};
```

The first vector corresponds to the temperature difference between target and actual reading.
The second vector is the power factor (0 to 100%) to apply for each (interpolated) delta.
