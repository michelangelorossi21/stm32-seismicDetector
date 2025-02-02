# Earthquake Detection System with STM32 🌍⚡
## Description
This project is based on the STM32 microcontroller, designed to detect seismic activity by monitoring vibrations. Using the integrated accelerometer and gyroscope sensors, the system detects ground movements and triggers a notification in case of seismic events. 🚨🌐

## Technologies Used:
- STM32 microcontroller 🖥️
- Always-on 3D accelerometer and 3D gyroscope LSM6DSL 📏
- C programming language 💻
- Data processing for vibration analysis 🔍
- [Serial Plotter](https://github.com/CieNTi/serial_port_plotter) to visualize vibrations
  
## Features:
- Real-time earthquake detection ⏱️
- Data acquisition from the accelerometer 
- Seismic event notification 📲
- Seismic vibrations visualization;
- Power-efficient design using low-power modes on STM32 ⚡💡
  
## How to Run the Project:
- Connect the STM32 board to the accelerometer sensor 🔌.
- Upload the firmware to the STM32 using STM32CubeIDE or similar tools 🛠️.
- Power the system and the device will continuously monitor vibrations 🌍.
- Seismic event detected: The system will process the data and, in case of a significant vibration, a signal or notification will be triggered 📡.

## Lessons Learned:
This project allowed me to deepen my knowledge of embedded systems, data acquisition, and vibration analysis. It also involved working with real-time data processing on microcontrollers, providing me with a solid foundation in both hardware and software development. 💡🔧
