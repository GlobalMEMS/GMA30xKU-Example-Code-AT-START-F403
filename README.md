AT-START-F403 + GMA30xKU example code
=====================================

Requirements
------------
- AT-START-F403 development board
- Sensor Fusion Arduino Daughter Board V1.0: GMA303KU is on the sensor board

I2C Connections
---------------
- Use I2C1
  - SCL: PB8
  - SDA: PB9
- GMA30xKU I2C 7-bit slave address: 0x18

Raw Data Sensitivity
--------------------
The sensitivity of the raw data output is normalized to 512 codes/g for all 3 axes.

Default gma30xku_initialization function in gma30xku.c
------------------------------------------------------
Default initialization steps:
 * Turn on offset temperature compensation
 * Set to continuous mode
 * Turn on low pass filter
 * Set data ready INT, ative high, push-pull

You may change the behavior of this initialization function to suit your purpose. Please refer to datasheet for more details on the register settings.

Usage of AutoNil
----------------
 * The program will do an offset AutoNil when executed. Hold the board steady and maintain in level facing up, then press **Key1** after the program prompt for input.
 * You may change the `DATA_AVE_NUM` macro in the gSensor_autoNil.h for the moving averae order for the offset estimation. Defautl is 32.
