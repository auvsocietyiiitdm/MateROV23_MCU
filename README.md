# MateROV23_MCU
Code to be deployed on the MCU on the ROV for MateROV2023 \
The code does the following:
  * Gets Acc, Mag and Gyro data from the Adafruit 9-axis IMU Sensor and logs it every 1 sec.
  * Everytime PWM Values are published in the topic 'pwm_values', they are given to the ESCs which control the thrusters.
\
# Note : 
  * If there is a dependency error of gccarm toolchain, add the following line in platformio.ini \
    `platform_packages = toolchain-gccarmnoneeabi@~1.90301.0`
  * If there is an error similar to `undefined reference to &Serial1` during compilation for STM32 Boards, make the following changes in the file located in: \
    **.pio/libdeps/Board Name/Rosserial Arduino Library/src/ArduinoHardware.h**
    1. Line 62: `#define SERIAL_CLASS USBSerial`
    2. Line 75: `iostream = &SerialUSB;`
