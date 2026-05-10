# 🚆 Railway Track Crack Detection & Monitoring Rover

### STM32 + ESP32 Autonomous Railway Safety System

![STM32](https://img.shields.io/badge/STM32-Nucleo_F446RE-blue?style=for-the-badge)
![ESP32](https://img.shields.io/badge/ESP32-Web_Server-orange?style=for-the-badge)
![Embedded C](https://img.shields.io/badge/Embedded-C-green?style=for-the-badge)
![IoT](https://img.shields.io/badge/IoT-Enabled-red?style=for-the-badge)
![Status](https://img.shields.io/badge/Project-Working-success?style=for-the-badge)

---

# 📌 Overview

Indian Railways operates across thousands of kilometers of railway tracks, where manual inspection is still heavily relied upon for detecting cracks, track displacement, and obstacles.

Even a small undetected surface crack can potentially lead to derailments, infrastructure damage, and safety risks.

To address this problem, this project implements an **autonomous railway inspection rover** using **STM32** and **ESP32** capable of:

* Detecting railway surface cracks
* Detecting nearby obstacles
* Stopping automatically during abnormal conditions
* Monitoring rover status in real time through a WiFi dashboard
* Transmitting live telemetry between controllers using UART communication

The rover combines **embedded systems**, **sensor interfacing**, **motor control**, and **IoT-based monitoring** into a compact railway safety prototype.

---

# 🚀 Key Features

## Embedded Control System (STM32)

* Real-time crack detection using IR sensor
* Obstacle detection using HC-SR04 ultrasonic sensor
* Autonomous motor control using L298N driver
* Immediate emergency stop on fault detection
* UART telemetry transmission
* Timer-based ultrasonic distance calculation
* Software PWM correction for motor balancing

---

## ESP32 IoT Monitoring System

* Local WiFi hotspot hosting
* Real-time browser dashboard
* UART communication with STM32
* Live system telemetry visualization
* Mobile-friendly responsive interface
* Event logging and alert system
* Local monitoring without internet dependency

---

# 🧠 System Architecture

```text
                 ┌──────────────────────┐
                 │   IR Crack Sensor    │
                 └──────────┬───────────┘
                            │
                 ┌──────────▼───────────┐
                 │ STM32 Nucleo F446RE │
                 │  Sensor Processing   │
                 │  Motor Control       │
                 │  UART Communication  │
                 └──────────┬───────────┘
                            │ UART
                            ▼
                 ┌──────────────────────┐
                 │        ESP32         │
                 │ WiFi + Web Server    │
                 │ Dashboard Hosting    │
                 └──────────┬───────────┘
                            │
                     WiFi Hotspot
                            │
                            ▼
                 ┌──────────────────────┐
                 │  Mobile Dashboard    │
                 │ Real-Time Monitoring │
                 └──────────────────────┘
```

---

# ⚙️ Hardware Components

| Component                 | Purpose                                |
| ------------------------- | -------------------------------------- |
| STM32 Nucleo F446RE       | Main embedded controller               |
| ESP32                     | WiFi communication + dashboard hosting |
| L298N Motor Driver        | DC motor driving                       |
| HC-SR04 Ultrasonic Sensor | Obstacle detection                     |
| IR Sensor Module          | Crack / surface break detection        |
| DC Motors + Chassis       | Rover movement                         |
| HC-05 Bluetooth Module    | UART testing & telemetry               |
| Li-ion Battery Pack       | Power source                           |
| Breadboard & Jumper Wires | Prototyping                            |

---

# 🔍 Working Principle

## 1. Rover Navigation

The rover continuously moves forward while monitoring track conditions.

---

## 2. Crack Detection

The IR sensor continuously checks the railway surface.

If:

* crack,
* surface break,
* or abnormal gap

is detected:

✅ STM32 immediately:

* stops the motors
* increments crack event counter
* sends alert data to ESP32

---

## 3. Obstacle Detection

The HC-SR04 ultrasonic sensor measures distance ahead.

If an object enters the unsafe threshold distance:

✅ Rover automatically stops.

---

## 4. Real-Time Dashboard Monitoring

ESP32 receives telemetry from STM32 through UART and hosts a local dashboard accessible through browser.

Users can monitor:

* ultrasonic distance
* crack status
* motor state
* WiFi signal
* alert conditions
* event counters
* distance history

in real time.

---

# 🌐 ESP32 Web Dashboard

## Dashboard Features

* Real-time ultrasonic distance gauge
* IR crack status (OK / CRACK)
* Motor state monitoring
* Obstacle detection alerts
* Crack event counters
* RSSI / WiFi signal strength
* Live status indicators
* Mobile responsive UI
* Local web hosting without internet

---

## Accessing Dashboard

### Connect to ESP32 Hotspot

```text
SSID: RailwayMonitor-AP
Password: railway123
```

### Open in Browser

```text
http://192.168.4.1:8000
```

---

# 🧩 Embedded Implementation Details

## STM32 Firmware

### TIM2 Microsecond Timer

Used for:

* ultrasonic pulse timing
* accurate echo duration measurement

---

### Distance Calculation

Distance is calculated using:

```text
Distance = (Echo_Time × Speed_of_Sound) / 2
```

Multiple samples are averaged to reduce sensor noise.

---

### Software PWM Balancing

One motor rotated slightly faster than the other, causing rover drift.

To solve this:

* software PWM duty cycle correction was implemented
* approximately 50% compensation applied

---

# 📊 Dashboard States

## Safe State

* No crack detected
* No obstacle nearby
* Rover moving normally

---

## Alert State

Triggered when:

* crack detected
* surface break detected
* obstacle too close

System:

* stops rover
* displays warning alerts
* updates counters
* changes dashboard status dynamically

---

# ⚠️ Challenges Faced

| Challenge                | Solution                      |
| ------------------------ | ----------------------------- |
| Motor imbalance          | Software PWM correction       |
| Ultrasonic noise         | Averaged multiple readings    |
| IR false triggering      | Threshold tuning              |
| Power instability        | Improved battery distribution |
| UART synchronization     | Optimized serial parsing      |
| ESP32 web responsiveness | Lightweight local dashboard   |

---

# 📈 Future Improvements

* AI-based crack classification
* GPS-based location tracking
* Cloud monitoring dashboard
* Camera integration
* Servo-based scanning mechanism
* Long-range wireless communication
* Railway analytics system
* Predictive maintenance algorithms

---

# 📁 Repository Structure

```text
STM32-Railway-Crack-Detection-System/
│
├── stm32_firmware/
│     └── main.c
│
├── esp32_dashboard/
│     └── railway_dashboard.ino
│
├── images/
│     ├── testing_on_track.jpg
│     ├── final_setup.jpg
│     ├── wiring.jpg
│     ├── dashboard_normal.jpg
│     └── dashboard_alert.jpg
│
├── README.md
└── LICENSE
```

---

# 📸 Project Demonstration

## 🚂 Working Prototype on Dummy Track

![Track](images/testing_on_track.jpg)

---

## 🔧 Final Hardware Setup

![Setup](images/final_setup.jpg)

---

## 🔌 Wiring & Sensor Integration

![Wiring](images/wiring.jpg)

---

# 🌐 ESP32 Dashboard Demonstration

## ✅ Normal Operating State

![Dashboard Normal](images/dashboard_normal.jpg)

---

## 🚨 Crack / Obstacle Alert State

![Dashboard Alert](images/dashboard_alert.jpg)

---

# 🎯 Learning Outcomes

This project helped in understanding:

* STM32 peripheral programming
* Embedded C development
* Timer configuration
* UART communication
* PWM motor control
* Sensor interfacing
* ESP32 networking
* Web dashboard hosting
* Real-time telemetry systems
* Embedded + IoT system integration

---

# 👨‍💻 Author

### Kunal Salunke

Electronics & Communication Engineering
MIT World Peace University

---

# 📜 License

This project is licensed under the MIT License.

```text
MIT License

Copyright (c) 2026 Kunal Salunke

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files to deal in the Software
without restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software.
```

---

# ⭐ Project Status

✅ Hardware Prototype Completed
✅ STM32 Firmware Working
✅ ESP32 Dashboard Integrated
✅ Real-Time Monitoring Functional
✅ Railway Crack Detection Demonstrated
