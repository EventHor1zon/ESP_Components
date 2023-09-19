# BME/P 280 Driver


### Version
Beta - all features working but limited

## Brief

This driver works for both the BME and BMP 280 Temperature/Pressure/Humidity sensor ICs. I decided to use a #define to add the functionality and register options of the more complex BME sensor. User should make sure to define BME_280 in the code or build options if using this device.

The driver init creates the driver task, handle and gets the device calibration data. The UpdateMeasurements function retrieves the latest data from the device and stores it in the handle. Calling the get function for temperature, pressure or humidity returns the latest retrieved sample. 

The measurements provided by the get functions for temp/pressure/humidity return the calibrated results in degrees C, hectopascals and percent relative humidity respectively.

The driver uses a task to update the latest measurements. The device can operate in normal mode, sampling at a given frequency (where the driver task sleeps and periodically polls for new measurements) or forced mode which is yet to be handled properly.

## Interface Type

I2C Interface

## How To Use

Initialise the driver using the init function. The user can specify the samples it wants to aquire in the init data struct. 


### TODOs

- Clean up task instantiation
- Fix notification to update using handle
- keep task state agnostic, allowing for multiple devices
- Deal with Cbuffer integration properly


## Datasheet

https://pdf1.alldatasheet.com/datasheet-pdf/view/1132060/BOSCH/BME280.html

## Credits

All mine, baby! Actually, the bosch datasheet has the conversion/calibration functions so yeah.
