# demo_supernova_icm42605

This repository contains a demonstration project for interfacing with an ICM42605 sensor using the Supernova host adapter connected to the I3C bus. The project showcases how to initialize the sensor, read accelerometer and gyroscope data in real-time, and visualize this data using `matplotlib` in Python.

![Sensor_Data_Visualization](https://github.com/binhollc/demo_supernova_icm42605/assets/947449/107e83d5-790f-44d2-81d0-73047f33b08d)

## Introduction

The project provides a Python script (`run.py`) that interfaces with an ICM42605 sensor. It demonstrates initializing the sensor, calibrating it, reading accelerometer and gyroscope data, and plotting this data in real time.

## Prerequisites

- Python 3.10
- Supernova host adapter
- ICM42605 sensor connected to the I3C bus

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/binhollc/demo_supernova_icm42605.git
   cd demo_supernova_icm42605
   ```

2. **Create and Activate a Virtual Environment:**

   It's recommended to create a virtual environment to manage dependencies.

   - On Windows:

     ```bash
     python -m venv venv
     .\venv\Scripts\activate
     ```

   - On macOS and Linux:

     ```bash
     python3 -m venv venv
     source venv/bin/activate
     ```

   You should now see `(venv)` in your command line, indicating that the virtual environment is active.

3. **Install Dependencies:**

   Use the provided `requirements.txt` to install the necessary Python packages.

   ```bash
   pip install -r requirements.txt
   ```

## Usage

Run the script using Python:

```bash
python run.py
```

The script will open a window displaying two real-time plots: one for accelerometer data and another for gyroscope data from the ICM42605 sensor. Press 'q' to exit the plot and stop the script.

To exit the virtual environment, use:

```bash
deactivate
```
