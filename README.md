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

- 
