# ESP_Components


A small library of my ESP-IDF/FreeRTOS based Components for
various devices. 

### Design

Each driver is written using ESP-IDF and FreeRTOS v9(?). 

Each driver follows a basic pattern

- Each driver has a task. This task is periodic if device mode requires, else waits on task notifications
- Each driver has an init function which should take a pointer to an init structure and return a handle to the driver structure or NULL
- Each driver should have an option to store handle on the heap. Using stack always better but some of these handles are 
    pretty chunky and the ESP32 has a lot of heap. Plus these handles are not really designed with re-use in mind
- Each driver controls a single device (lookin at you, WS2812b driver!)
- Each driver has a set of public Getter/Setter functions, which take as arguments
a device handle and a pointer to a value (set) or value storage (get)
- Each driver contains a map of parameters for use with my peripheral manager (see ESPHome for more deets)
- Each driver should have a TaskHandle for notifications



### Current Drivers 

- A4988 Stepper motor IC         (beta)
- APA102 Addressable LEDs        (beta)
- APDS Color/Prox/Gesture sensor (beta)
- BM(P|E) 280 Temperature sensor (ok)
- CircularBuffer                 (beta - now with packet storage!)
- CBuffer_Utils                  (not complete)
- CCS811 Gas sensor              (not complete)
- DS2321 RTC IC                  (beta)
- Futaba VFD                     (not working)
- HMC5883 Magnetometer           (beta)
- HPDL1414 Bubble LED display    (beta)
- LedEffects for ws2812/apa102   (beta)
- LoRa SX1276 chipset            (beta)
- LSM6DS3  Accel/Gyro sensor     (ok)
- Max30102 Heartbeat sensor      (not complete)
- MFRC552 NRFC/MiFare Transciever (alpha)
- MSGEQ7 Audio spectrum ic       (alpha)
- PushButton driver              (ok)
- RotaryEncoder driver           (ok)
- RGB 3 * PWM channel driver     (beta)
- SIIM800L Driver                (not even started)
- SSD1306 OLED screen            (alpha) (credit to https://github.com/yanbe/ssd1306-esp-idf-i2c.git)
- ST7735 Screen                  (not complete)
- ESP32 System interface         (not complete)
- VEML6070 UV Sensor IC          (beta)
- VL53L0X Time of flight sensor  (beta) (credit to ST's VL53L0X api - included for ease of use)
- WS2812 Addressable LEDs        (beta) (credit to ESP-IDF's WS2812 example)
- Utilities (various bits)       (beta) 


### Informations

Some of these drivers built with inspiration or borrowed sections from other's code. Credit where it's due. 

## TODOs

- Driver handles are chunky and drivers are often disorganised and bloated. Slim them down!
- Refactor drivers with simplicity in mind. Event/Interrupt based task actions. Let users implement polling if neccesary.
- Replace all driver heap allocation with stack where possible. Also consider enabling heap with preprocessor config _DONE_
- Any number of driver instances will use a single task so remember to keep tasks state-agnostic and rely on the handle
- Also remember that the task only needs to be initialised ONCE. Check for running task?
- TaskHandle_t task can be static to the driver file
- Careful with task operations like sleep/standby etc!


Refactors:

- APA102: Don't include spi init code. Simplify the ledfx struct & store as member not pointer.