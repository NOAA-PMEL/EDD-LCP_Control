# LCP Control Requirements

## Introduction
This document captures the electrical and firmware requirements of the Low-Cost Profiler.



## Electrical Requirments



## Firmware Requirements
### Sensors
- [ ] Pressure sensor
- [ ] Temperature sensor
- [ ] GPS
- [ ] Accelerometer
  - [ ] Double tap recognition
  - [ ] Tilt 
  - [ ] Oscillation

### COMs
- [ ] UART 
- [ ] I2C
- [ ] SPI
- [ ] Iridium 
- [ ] Bluetooth


### Analog
- [ ] Bus voltage monitoring

### Low Power Mode
- [ ] Low Power Mode

### Application Level
  - [ ] Console
    - [ ] Set
      - [ ] See Config Parameters
    - [ ] Get 
      - [ ] See Config Parameters
    - [ ] Stream data
      - [ ] Debug
      - [ ] Warnings
      - [ ] Errors
      - [ ] All
  - [ ] Config
    - [ ] Bluetooth Config
    - [ ] Other?
  - [ ] Control Mode
    - [ ] Dive Mode
    - [ ] Park Mode
    - [ ] Dive 2 Mode
    - [ ] Profile Mode
    - [ ] Surface Mode
    - [ ] Transmit Mode
  - [ ] Deployment Mode
    - [ ] Storage Mode
    - [ ] Config Mode
    - [ ] Deploy Mode
    - [ ] Seek Mode
    - [ ] Profile Mode
    - [ ] Moored Mode
    - [ ] Pop-up Mode
  - [ ] Motion Control
    - [ ] State/Estimation
    - [ ] Current state
    - [ ] Piston Control
  - [ ] Data Acquisition
    - [ ] Depth Estimation
    - [ ] Chemistry
      - [ ] Temperature
      - [ ] CTD [Future] 
      - [ ] Chlor-A [Future]
      - [ ] PAR [Future]
    - [ ] Position/Tilt
      - [ ] GPS
      - [ ] Accleration
      - [ ] IMU
  - [ ] RTOS
    - [ ] Modes
    - [ ] Threads
    - [ ] Scheduling
    - [ ] Queues