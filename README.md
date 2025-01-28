# Thermal Camera and Distance Integration Project

## Index

- [Overview](#overview)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
- [Usage](#usage)
  - [Selecting ROI](#selecting-roi)
  - [Running the Script](#running-the-script)
  - [Saving Data](#saving-data)
- [System Overview](#system-overview)
  - [Circuit Schematics](#circuit-schematics)
  - [Hardware Development](#hardware-development)
  - [Software Overview](#software-overview)
  - [Communications Overview](#communications-overview)
- [Limitations and Future Improvements](#limitations-and-future-improvements)
- [Demo](#demo)
- [Authors](#authors)
- [References](#references)

---

## Overview

This project combines a thermal camera and an ESP32 to capture and analyze thermal data and distance measurements. It was developed as part of the **YAKISUGI Torch Project**, a system designed to optimize and standardize the traditional Japanese wood-burning technique for enhanced durability, pest resistance, and waterproofing. 

**About the YAKISUGI Method:**
The YAKISUGI method is a traditional Japanese wood preservation technique that involves charring the surface of the wood with fire. This process results in increased durability, pest resistance, and waterproofing.

**Objective and Motivation:**
- Optimize the wood-burning process by providing visual feedback, measurements, and real-time data.
- Preserve and modernize this ancestral technique for artisans and robotic systems.
- Enable interaction between digital devices and artisans, informing future designs for robotic systems.

The thermal camera streams real-time data, allowing users to select a Region of Interest (ROI), while the ESP32 provides proximity data. The script integrates both sources of information, displays them on an interactive interface, and saves them to a CSV file.

**Main Features:**

- Real-time thermal data processing.
- Integration with an ESP32 for distance measurement.
- Interactive user interface for ROI selection.
- Data logging of temperature and distance values.
- Preservation of the YAKISUGI technique through standardized and optimized burning processes.
- 
![HARDWARE II_Final_Mau Javi Charlie_Page_03](https://github.com/user-attachments/assets/d9456d1b-4fae-4dc0-aa87-f2f4a1cfb60c)
![HARDWARE II_Final_Mau Javi Charlie_Page_04](https://github.com/user-attachments/assets/d41eb06c-5659-402f-ac5c-84f473ee76b9)
![HARDWARE II_Final_Mau Javi Charlie_Page_05](https://github.com/user-attachments/assets/8840985c-dd4d-45ea-a093-15970bed9edc)

---

## Getting Started

### Prerequisites

Ensure the following are installed:

- **Ubuntu 20.04 LTS** or later.
- **Python 3.7** or newer.
- **Compatible Thermal Camera** (e.g., FLIR USB camera).

### Dependencies

Install the required Python libraries:

```bash
pip3 install numpy opencv-python requests
```

Additional system libraries might be needed depending on your thermal camera:

```bash
sudo apt-get install libuvc-dev
```

### Installation

1. Clone this repository:

```bash
git clone https://github.com/your-repository-url.git
cd your-repository-folder
```

2. Verify your ESP32 is set up and running with its provided firmware.

   - The ESP32 should serve distance data on a network-accessible IP (default: `http://192.168.4.1`).

3. For additional guidance on using the thermal camera, refer to the repository developed for this purpose: [Thermal Camera GitHub Repository](https://github.com/Clarrainl/thermal-camera).

---

## Usage

### Selecting ROI

1. Run the script:

   ```bash
   python3 thermal_camera_esp32.py
   ```

2. The live feed from the thermal camera will appear.

3. Press `s` to select the Region of Interest (ROI):

   - Use your mouse to drag and select a rectangular area.
   - This area will be analyzed for temperature data.

### Running the Script

- Once the ROI is selected, the script will:

  - Continuously process thermal data within the ROI.
  - Query the ESP32 for distance measurements.
  - Display the maximum and minimum temperatures, as well as the distance, in real-time.

- Press `q` at any time to exit the program.

### Saving Data

- The script logs temperature and distance data at regular intervals (default: every 5 seconds).
- After exiting, you will be prompted to save the data to a CSV file:
  - Example output file: `temperature_distance_data.csv`.

---

![HARDWARE II_Final_Mau Javi Charlie_Page_04](https://github.com/user-attachments/assets/601022fb-d11f-4555-898d-ccff1ccb585c)


## System Overview

![HARDWARE II_Final_Mau Javi Charlie_Page_06](https://github.com/user-attachments/assets/44528cb1-e4eb-493f-9d66-733e3fa69011)

The system integrates various hardware and software components to achieve real-time feedback and optimization of the wood-burning process:

![HARDWARE II_Final_Mau Javi Charlie_Page_08](https://github.com/user-attachments/assets/c2c50654-38b8-42be-a692-13f2589020e9)

**Hardware Components:**
- **Thermal Camera:** FLIR Lepton (PureThermal 2 Cam) for capturing temperature data.
- **ESP32:** Microcontroller for distance measurement and Wi-Fi communication.
- **Ultrasonic Sensor:** HC-SR04 for proximity sensing.
- **OLED Screen:** 0.96-inch Waveshare for real-time display of temperature, time, and distance.
- **Battery and Voltage Regulator:** Power supply for the ESP32 and peripherals.

![HARDWARE II_Final_Mau Javi Charlie_Page_09](https://github.com/user-attachments/assets/7dd5ed37-7714-40c1-8efc-59d7123bcc90)

**Software Features:**
- **Thermal Data Processing:** Captures and processes temperature data, converting it into Celsius and generating color-mapped thermal images.
- **Distance Measurement:** Queries the ESP32 via HTTP to retrieve real-time distance data.
- **Real-Time Visualization:** Displays temperature and distance on an interactive interface, allowing artisans to optimize the burning process.
- **Data Storage:** Logs time, temperature, and distance in a CSV file for post-process analysis.

### Circuit Schematics

This system integrates multiple hardware components:

![HARDWARE II_Final_Mau Javi Charlie_Page_07](https://github.com/user-attachments/assets/9df3b319-22db-4b94-bb63-941bb58b6686)

1. **Ultrasonic Sensor (HC-SR04):** Measures distance to ensure proper torch positioning.
2. **ESP32 Microcontroller:** Manages data from the ultrasonic sensor and streams it to Python via HTTP.
3. **OLED Screen:** Displays real-time data to the user.
4. **PureThermal Camera:** Captures and streams thermal data.

---

### Hardware Development

**Working Principles:**
- **Thermal Capture:** Measures surface temperature of the wood using a thermal camera.
- **Distance Measurement:** Ensures consistent torch positioning through an ultrasonic sensor.
- **Data Processing:** ESP32 processes time, temperature, and distance data and displays it in real-time on the OLED screen.

### Software Overview

**Libraries Used:**
- **OpenCV & NumPy:** For image processing and numerical computation.
- **uvctypes:** Communication with the thermal camera.
- **Queue:** Manages real-time thermal data buffering.
- **CSV:** Logs processed data for later analysis.
- **Requests:** Queries the ESP32 for distance measurements via HTTP.

### Communications Overview

**Protocols Used:**
- **USB Video Class (UVC):** Streams thermal frames from the PureThermal camera.
- **HTTP:** ESP32 hosts a server, and Python retrieves distance data in JSON format.
- **Trigger-Echo:** Ultrasonic sensor sends pulses and calculates distance using timing.

![HARDWARE II_Final_Mau Javi Charlie_Page_10](https://github.com/user-attachments/assets/0a7f5c3e-fb7f-4c57-9225-09b55a3cc031)
![HARDWARE II_Final_Mau Javi Charlie_Page_11](https://github.com/user-attachments/assets/f5421f9b-fef8-42bc-b4f3-9b38471f85a4)
![HARDWARE II_Final_Mau Javi Charlie_Page_12](https://github.com/user-attachments/assets/a67c52ce-6442-47b0-87dd-df9495f6c821)

---

## Limitations and Future Improvements

### Limitations
- The resolution of the thermal camera limits precision for detailed temperature mapping.
- Ultrasonic sensors are sensitive to surface inconsistencies, affecting accuracy.
- The OLED screen size constrains the amount of real-time data displayed.

### Future Improvements
- Upgrade to a higher-resolution thermal camera for detailed mapping.
- Replace the ultrasonic sensor with LiDAR for enhanced precision.
- Integrate data with mobile or desktop applications for better visualization.
- Add ambient condition sensors for humidity or other environmental factors.
- Implement audio alerts to enhance safety during burning.

![HARDWARE II_Final_Mau Javi Charlie_Page_14](https://github.com/user-attachments/assets/d86907ca-277d-41e1-b2c2-265a7082e020)

---

## Demo

To demonstrate the script:

1. Connect your thermal camera and ESP32.
2. Run the script as described in [Usage](#usage).
3. Observe the real-time feed and the logged data in the CSV file.

The results include a table with:

| Time (HH\:MM\:SS) | Min Temp (C) | Max Temp (C) | Distance (cm) |
| ----------------- | ------------ | ------------ | ------------- |
| 00:00:05          | 20.5         | 37.8         | 15            |

---

## Authors

- [Charlie Larraín](https://www.linkedin.com/in/charlie-larrain/) - Developer
- [Javi Albo](https://www.linkedin.com/in/javi-albo/) - Developer
- [Mau Weber](https://www.linkedin.com/in/mau-weber/) - Developer

*This project was developed as part of the Master in Robotics and Advanced Construction (MRAC), Term II, 2024/25.*

---

## References

- [FLIR Camera Documentation](https://www.flir.com/support/)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [Thermal Camera GitHub Repository](https://github.com/Clarrainl/thermal-camera)
- **Presentation Document:** "Hardware II Final Presentation - YAKISUGI Torch Project."

---
