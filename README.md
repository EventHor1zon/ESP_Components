# ESP_Components


A small library of my ESP-IDF/FreeRTOS based Components for
various devices. 

### Design

Each driver is written using ESP-IDF and FreeRTOS v9(?). 

Each driver follows a basic pattern

- Each driver has a task. Task should wait on input from queue or notification. Let users implement polling approaches.
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
- APDS Color/Prox/Gesture sensor (beta - all basics working. Refactor for events/interrupts)
- BM(P|E) 280 Temperature sensor (Release - TODO: Refactor to fix naming conventions, remove polling mode)
- CircularBuffer                 (beta - now with packet storage!)
- CBuffer_Utils                  (unused)
- CCS811 Gas sensor              (barely started - how do I turn the readings into gas measurements???)
- DS2321 RTC IC                  (beta - some ops looking wrong, require testing and alarms)
- Futaba VFD                     (beta - could use some testing/streamlining but fully working)
- HMC5883 Magnetometer           (beta - all basics working)
- HPDL1414 Bubble LED display    (alpha - never seen working)
- LedStrand Driver               (alpha - needs testing/tweaking) 
- LedEffects for ws2812/apa102   (beta - three working effects! - In refactor)
- LoRa SX1276 chipset            (beta - Tx working. Get another module for Rx development)
- LSM6DS3  Accel/Gyro sensor     (Release - TODO: Refactor for simplicity!)
- Max30102 Heartbeat sensor      (beta) - Working but not well tested
- MFRC552 NRFC/MiFare Transciever (alpha)
- MSGEQ7 Audio spectrum ic       (beta - could do with tweaking/tidying)
- PushButton driver              (Release - TODO: Refactor for static task/multiple instances)
- RotaryEncoder driver           (Release - TODO: As above )
- RGB 3 * PWM channel driver     (beta)
- SIM800L Driver                 (not even started)
- SSD1306 OLED screen            (alpha - needs work on scrolling/cursor position) (credit to https://github.com/yanbe/ssd1306-esp-idf-i2c.git)
- ST7735 Screen                  (not complete)
- ESP32 System interface         (not complete)
- VEML6070 UV Sensor IC          (beta)
- VL53L0X Time of flight sensor  (beta) (credit to ST's VL53L0X api - included for ease of use TODO: Change this to a subrepo)
- Utilities (various bits)       (beta)


### Informations

Some of these drivers built with inspiration or borrowed sections from other's code. Credit where it's due. 

## TODOs

- Driver handles are chunky and drivers are often disorganised and bloated. Slim them down!
- Refactor drivers with simplicity in mind. Event/Interrupt based task actions. Let users implement polling if neccesary.
- Replace all driver heap allocation with stack where possible. Also consider enabling heap with preprocessor config _DONE_
- Any number of driver instances will use a single task so remember to keep tasks state-agnostic and rely on the handle
    - this may involve replacing task notifications with message queues in order to pass the driver handle to the task
- Also remember that the task only needs to be initialised ONCE. Check for running task?
- TaskHandle_t task can be static to the driver file
- Careful with task operations like sleep/standby etc!


## Refactors

- APA102: Don't include spi init code. Simplify the ledfx struct & store as member not pointer.
- Seeing as how I'm moving towards a generic read/write for effects, could it be worth merging both addressable led drivers into on generic driver?
- WS2812B: A very early driver. Apply learnt tricks, refactor similarly to APA102