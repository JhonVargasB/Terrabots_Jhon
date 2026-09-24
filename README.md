# Terrabots — Embedded Systems & Robotics Workspace

Repository containing firmware developed during robotics and embedded-systems work with **ESP-IDF**.

It includes progressive hardware experiments, sensor integration, motor-control exercises and the `Robert_Rojo` firmware workspace.

## Repository structure

```text
Terrabots_Jhon/
├── docs/
│   ├── Datasheets/
│   └── Guías/
└── firmware/
    ├── semana_01/
    │   ├── ESP32_HolaMundo/
    │   ├── TestBME280/
    │   ├── TestMPU6050/
    │   └── TestMPU9250/
    ├── semana_04/
    │   └── PWM_Control/
    ├── semana_05/
    │   └── PID_ctrl/
    └── Robert_Rojo/
```

## Topics covered

- ESP32 / ESP-IDF
- Environmental sensors
- IMUs
- GPS
- PWM motor control
- PID control
- Data storage
- USB communication
- Modular embedded firmware

## Robert_Rojo

The `firmware/Robert_Rojo` directory contains a larger ESP-IDF project with modular components and individual hardware test programs for subsystems such as:

- IMU
- GPS
- Pressure / environmental sensing
- Storage
- USB CDC communication

## Purpose

This repository is kept as a development record of embedded-system experiments and integration work. Individual projects may later be separated into dedicated repositories as they mature.

---

**Author:** Jhon Vargas  
Electronic Engineering student
