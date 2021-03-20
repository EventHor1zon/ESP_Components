# ESP_Components


A small library of my ESP-IDF/FreeRTOS based Components for
various devices. 

### Design

Each driver is written using ESP-IDF and FreeRTOS v9(?). 

Each driver follows a basic pattern

- Each driver has a task. This task is periodic if device mode requires, else waits on task notifications
- Each driver has an init function which returns a handle to the driver structure or NULL
- The driver structures are stored in allocated heap memory
- Each driver should control a single device (lookin at you, WS2812b driver!)
- Each driver should have a TaskHandle for notifications



### Current Drivers 

- APA102 Addressable LEDs        (alpha)
- BM(P|E) 280 Temperature sensor (beta)
- CCS811 Gas sensor              (not complete)
- HPDL1414 Bubble LED display    (beta)
- LSM6DS3  Accel/Gyro sensor     (alpha)
- LoRa SX1276 chipset            (in-dev)
- Max30102 Heartbeat sensor      (not complete)
- PushButton driver              (beta)
- RotaryEncoder driver           (beta)
- SSD1306 OLED screen            (not complete)
- ST7735 Screen                  (not complete)
- ESP32 System interface         (not complete)
- WS2812 Addressable LEDs        (beta)
- MSGEq7 Audio spectrum ic       (not complete)
- VL53L0X Time of flight sensor  (alpha)
