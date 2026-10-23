# 🚀 Intelligent PWM-Based BLDC Motor Drive with Real-Time Condition Monitoring, Energy Analytics and Predictive Maintenance

> Industrial IoT-based BLDC motor monitoring system using **ESP32**, **Blynk IoT**, **MATLAB/Simulink**, and multiple sensors for real-time condition monitoring, fault detection, energy analytics, and predictive maintenance.

---

## 📌 Project Overview

Industrial motors are widely used in manufacturing, automation, HVAC systems, and electric vehicles. Unexpected motor failures can lead to production downtime and maintenance costs.

This project develops an intelligent BLDC motor monitoring system that continuously measures electrical and mechanical parameters, analyzes motor health, detects abnormal operating conditions, and predicts maintenance requirements using real-time sensor data.

The system combines embedded hardware, IoT cloud monitoring, OLED visualization, and MATLAB simulation into one integrated solution.

---

# ✨ Features

- ✅ Real-Time Voltage Monitoring
- ✅ Real-Time Current Monitoring
- ✅ Power Monitoring
- ✅ Energy Consumption Analysis
- ✅ Temperature Monitoring (DS18B20)
- ✅ Vibration Monitoring (MPU6050)
- ✅ Motor Health Score Calculation
- ✅ Fault Detection
- ✅ Predictive Maintenance
- ✅ OLED Display Interface
- ✅ Blynk IoT Dashboard
- ✅ MATLAB/Simulink Validation

---

# 🛠 Hardware Components

| Component | Purpose |
|-----------|---------|
| ESP32 DevKit V1 | Main Controller |
| BLDC Motor | Industrial Load |
| BLDC Driver | Motor Drive |
| ACS712 | Current Measurement |
| Voltage Sensor | Voltage Measurement |
| DS18B20 | Temperature Monitoring |
| MPU6050 | Vibration Monitoring |
| OLED SSD1327 Display | Local Display |
| 24V SMPS | Power Supply |
# 📸 Project Gallery

## Circuit Diagram
![Circuit](Circuit_Diagram/Circuit_Diagram.png)



## Hardware Setup
![Hardware](Hardware/Hardware_Setup.jpg)




---

# 💻 Software Used

- Arduino IDE
- ESP32 Board Package
- Blynk IoT Cloud
- MATLAB/Simulink
- GitHub

---

# 📊 Parameters Monitored

- Voltage (V)
- Current (A)
- Power (W)
- Energy (Wh)
- Temperature (°C)
- Vibration (m/s²)
- Motor Health Score (%)
- Performance Index
- Reliability Index

---

# ⚠ Fault Detection

The system detects:

- Over Voltage
- Under Voltage
- Over Current
- High Temperature
- Excessive Vibration
- Motor Overload

---

# 🔧 Predictive Maintenance

The predictive maintenance algorithm continuously evaluates the motor condition based on sensor readings.

Health Score is calculated using:

- Temperature
- Current
- Voltage
- Vibration

If the health score falls below the predefined threshold, the system recommends maintenance before a complete failure occurs.

---

# 📱 Blynk IoT Dashboard

The Blynk dashboard displays:

- Live Voltage
- Live Current
- Live Power
- Energy Consumption
- Temperature
- Vibration
- Health Score
- Fault Status
- Performance Index


---

# 📈 MATLAB/Simulink

The MATLAB model is used to validate:

- BLDC Motor Performance
- PWM Drive
- Energy Analysis
- Motor Characteristics

Simulation files are available in the **MATLAB** folder
.
## MATLAB Simulation
![Simulation](MATLAB/Simulink_Model.png)
---

# 📂 Repository Structure

```
BLDC-Motor-Condition-Monitoring-System
│
├── Arduino_Code
│   └── BLDC_Monitor.ino
│
├── MATLAB
│   ├── BLDC_Model.slx
│   └── Simulation Outputs
│
├── Hardware
│   └── Hardware Images
│
├── Circuit_Diagram
│   └── Circuit Images
│
├── Images
│   └── Blynk Dashboard & Results
│
├── Presentation
│   └── Project PPT
│
├── Project_Report
│   └── Final Report
│
├── README.md
└── LICENSE
```

---

# 🎯 Applications

- Industrial Motor Monitoring
- Smart Manufacturing
- Industry 4.0
- Predictive Maintenance
- Factory Automation
- Energy Management
- Electric Vehicle Motor Monitoring

---

# 🚀 Future Improvements

- AI-Based Fault Prediction
- Machine Learning for Failure Classification
- Remaining Useful Life (RUL) Estimation
- Email/SMS Alerts
- Cloud Database Integration
- Mobile Application

---

# 👨‍💻 Author

**Mellacheruvu Sadasiva Surya Srinivas**

B.Tech – Electrical and Electronics Engineering

Amrita Vishwa Vidyapeetham, Coimbatore

---

## ⭐ If you like this project, consider giving it a Star!
