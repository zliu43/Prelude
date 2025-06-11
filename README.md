🕰️ Digital Pocketwatch

A modern twist on a timeless accessory — this Digital Pocketwatch is a wearable device built on a custom PCB, featuring motion sensing, a high-quality touch display, and real-time clock functionality. Powered by the ESP32-S3, it integrates advanced graphics and sensor-driven interactivity in a compact form factor.
🔧 Features

    ESP32-S3 MCU
    Dual-core processor with Wi-Fi and Bluetooth LE support, ideal for low-power wearable applications.

    18-bit RGB Touch Display
    Driven by the Bridgetek FT813 graphics engine for high-resolution GUIs, capacitive touch input, and smooth animations.

    IMU (Inertial Measurement Unit)
    Enables step counting, motion recognition, and potential gesture input using accelerometer and gyroscope data.

    RTC (Real-Time Clock)
    Maintains accurate timekeeping even in low-power modes or when the device is turned off.

    Custom PCB
    Designed for wearability and optimized power and space management.

🚀 Project Goals

    Create an open-source wearable that bridges classic aesthetics with smart functionality.

    Provide a clean, responsive UI on a small form-factor screen.

    Explore motion-based interactions using the onboard IMU.

    Demonstrate low-power techniques with deep sleep, RTC wakeups, and peripheral management.

📦 Hardware Overview
Component	Description
ESP32-S3	Main microcontroller (Wi-Fi + BLE capable)
FT813	18-bit RGB touch display controller
IMU	Step detection and motion tracking
RTC	Low-power timekeeping module
Custom PCB	Integrates all components in wearable format
🖼️ Display

    Type: Capacitive touch, 18-bit RGB

    Driver: FT813 by Bridgetek

    UI Capabilities:

        Rich fonts and icons

        Touch-driven menus and animations

        Custom watch faces

📐 Motion Features

    Step counter with configurable thresholds

    Activity recognition: detect walking, idle, or gestures

    IMU integration for real-time movement data logging

🔌 Power Management

    Deep sleep support with RTC wake

    IMU-based wake-on-motion

    Optimized for wearable battery life
