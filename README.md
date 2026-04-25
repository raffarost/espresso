## Espresso Machine PID controller using ESP32 and RainMaker

### Brief description

This project implements a PID/P controller to effectively control the temperature of an Espresso machine boiler, for a stable setpoint.
Another feature implemented is the control of the water pump and pre-infusion settings. Combined, these functionalities allow for
improved espresso extraction and consistency.

**Firmware version** is defined in the repository root file [`VERSION`](VERSION); CMake and the legacy `Makefile` set `PROJECT_VER` from it, and the running image reports it via ESP-IDF (`esp_app_get_description()->version`).

April 2026 — https://github.com/raffarost/espresso

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

```c
/*    delta °C:  -10   0  0.5   1   2   4   10   25   50   70  */
static float   deltaBkp[BKP_NUM] = {-10,   0,  0.5,   1,   2,   4,   10,   25,   50,   70};
static float controlSet[BKP_NUM] = {  0,   0,    5,   5,   5,   8,   15,   30,   80,  100};
```

The first vector is the temperature difference between setpoint and actual reading (°C).
The second vector is the driver power value (0–100) applied for each (interpolated) delta.

##### TRIAC driver hardware constraint

The dimmer driver uses phase-angle control with a **fixed gate pulse width of 4 timer steps**.
For driver values 1–4, the gate pulse extends beyond the AC half-cycle boundary, re-latching
the TRIAC at the start of the next half-cycle and delivering ~25 % average power regardless
of the intended setting.

**Minimum safe value in `controlSet[]` is 5. Never use values 1–4.**  
Use 0 (heater fully off) or ≥ 5.

##### Actual power delivery vs. driver value

Phase-angle control is highly nonlinear. The driver value does **not** map linearly to
delivered power — most of the useful range is concentrated above 20.

| Driver value | Approx. actual power (% of rated) |
|---|---|
| 5 | ~0.1 % |
| 10 | ~0.6 % |
| 15 | ~2 % |
| 20 | ~5 % |
| 25 | ~9 % |
| 30 | ~15 % |
| 40 | ~31 % |
| 50 | ~50 % |
| 60 | ~69 % |
| 80 | ~95 % |
| 99 | ~100 % |

Values below ~15 deliver negligible heat and are only useful in `controlSet[]` as a defined
floor to avoid the gate-overflow bug (see above).  Practical maintenance and warmup
calibration should use values in the 15–99 range.

#### Pump heat buffer calibration

When the pump is active (pre-infusion, brew, or flush), cold water entering the boiler causes a
temperature drop. To compensate, a time-indexed power profile is applied instead of the idle
temperature controller:

```
static int pumpOnHeatBuff[PUMP_ON_HEAT_BUFF_LEN] = {
    100, 100, 100, 100,  80,  80,  40,  40,  30,  30,  30,  40,  40,  40,  40,  60,  60,  60,  60,  60
};
```

Each entry is the heater power (0–100%) applied at the corresponding second of pump-on time.
Index 0 is the first second, index 1 the second, and so on up to `PUMP_ON_HEAT_BUFF_LEN - 1`.
The vector length must be at least as long as the maximum configured brew time and flush time.

##### Extraction flow dynamics

The power profile follows the natural resistance the coffee puck offers to water flow during extraction:

- **Phase 1 (~4 s)** — Free-flow: water saturates the dry puck with little resistance. Flow rate is high and the boiler cools quickly, so high heater power is needed.
- **Phase 2 (~4 s)** — Puck compressing: swelling grounds start restricting flow. Less water moves through, so less compensation is required.
- **Phase 3 (~6 s)** — Maximum compression: the puck is fully saturated and tightly packed. Flow is most restricted and the boiler loses less heat, so power demand is at its lowest.
- **Phase 4 (~6 s+)** — Puck deterioration: channels begin to form as the grounds break down, flow gradually recovers, and power demand rises again.

This is a general profile — it can vary significantly depending on basket type (single vs. double) and dimensions, coffee dose, grind size, and temperature sensor position within the machine.
