# 🩺 Healthcare Monitoring Gadget

This project involves the design and implementation of a healthcare monitoring system that collects essential patient vitals through a wearable gadget and displays them in real-time on a web application. The system is intended to help doctors and caregivers remotely monitor patients, especially those with chronic conditions or limited mobility.

---

## 📌 Overview

The healthcare system is composed of two main components:

1. **Embedded Gadget**: A microcontroller-based wearable device that continuously collects biometric data.
2. **Web Application**: An interactive dashboard where medical professionals can access, visualize, and track historical health data.

---

## ⚙️ Features

- Real-time collection of vital signs:
  - Body temperature
  - Blood pressure
  - SpO₂ (blood oxygen level)
  - Heart rate

- Wireless data transmission from the gadget to the web application
- Intuitive web interface for:
  - Live monitoring
  - Historical charts and trends
  - Remote patient tracking

---

## 🧰 Tech Stack

### Embedded System:
- **ESP32**: WiFi-enabled microcontroller for data acquisition and communication
- **Sensors**:
  - **MAX30205**: Temperature sensor
  - **MAX30102**: Pulse oximeter and heart-rate sensor

### Backend:
- **Python** + **FastAPI**: RESTful API to receive and serve patient data
- **PostgreSQL**: Relational database to store time-series biometric data

### Frontend:
- **React.js**: Responsive interface for data visualization and interaction
- **Chart.js / D3.js** *(optional)*: Used to plot time-based health metrics


