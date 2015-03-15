# FreematicsBaseline

A "lite" version of the firmware available for the Freematics OBD data logger
It is greatly inspired (and mostly cloned) from Stanley Huang's work available at https://github.com/stanleyhuangyc/Freematics.

This firmware is experimental and was build to address the following requirements.

- It logs GPS and NEMS data in a single line in CSV files.
- It DOES NOT read any CAN/OBD bus data

It uses the following libraries :

SoftwareSerial
Wire
SPI
SD
I2Cdev
MPU9150
