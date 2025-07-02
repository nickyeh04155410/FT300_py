# FT300_py
# FT300 Force/Torque Sensor UI
**User Guide & Operating Instructions**

---

## Overview

This application provides a real-time graphical interface for viewing and recording data from the Robotiq FT300 force/torque sensor. Data is updated at 100 Hz, displayed as plots, and can be saved for later analysis.

---

## 1. Selecting the Sensor COM Port

- Connect your FT300 sensor to the PC via USB and ensure it is powered (24V supply required).
- At the top of the window, locate the **“COM port”** drop-down menu.
- Select the correct COM port corresponding to your FT300 sensor.
    - If unsure, unplug and replug the sensor and note which port appears or disappears.
    - If your desired port is not shown, click the refresh (↻) button next to the drop-down menu to update the list of available ports.

**Start Data Acquisition:**
- Press the **Start** button to begin reading sensor data from the selected COM port.
- The button label will change to **Stop** when acquisition is active.

> **Important:**  
> - If you select the wrong COM port, no sensor data will be displayed or recorded.
> - Double-check your selection. The program does not warn if the port is incorrect; it simply does not show live data.
> - Only one program can connect to the FT300 sensor at a time.

---

## 2. Automatic Zero Offset

- Each time you start acquisition, the program automatically tares (zeroes) the sensor.
- The force/torque values at startup are set as the reference zero.
- No manual zeroing is required. All displayed and saved values are relative to this offset.

---

## 3. Live Data Display

- **Upper plot:** Three-axis force data (Fx, Fy, Fz) in Newtons (N).
- **Lower plot:** Three-axis torque data (Tx, Ty, Tz) in Newton-meters (Nm).
- Data updates every 0.01 seconds (100 Hz) for smooth, real-time monitoring.
- Plots are hidden until acquisition is started.

---

## 4. Saving Data

- **Set Save Location:**  
    Click the **Browse...** button to choose or create a CSV file for saving your data.

- **Start Recording:**  
    Click the **Save** button to begin recording sensor data. The button will change to **Stop Save** while recording.

- **While Recording:**  
    All new sensor data will be stored in memory for later saving.

- **Stop and Save:**  
    Click **Stop Save** to end recording. All collected data will be written to the selected CSV file.
    - You will see a confirmation message with the file path and the number of samples saved.

---

## 5. Y-Axis Scaling Controls

- **Fixed Y-Axis:**  
    Click to set:
    - Force: -20 to +20 N
    - Torque: -2 to +2 Nm

- **Auto Y-Axis:**  
    Click to enable automatic scaling based on current data.

---

## 6. Exiting the Program

- Simply close the window to exit.
- All background sensor processes will terminate automatically.

---

## Troubleshooting

### No Data Displayed
- Check that you have selected the correct COM port.
- Ensure your FT300 sensor is connected and powered (24V supply).
- Only one instance of the program or connection can access the sensor at a time.

### COM Port Not Found
- Click the **refresh** button.
- Reconnect the sensor’s USB cable and look for the port that appears.

### Data Not Saved
- Ensure you have selected a valid save location.
- Confirm that you clicked **Stop Save** after recording.

---

## Contact

If you have questions or encounter issues, please contact:

**Yu-Peng Yeh**  
Email: [nickyeh0415@gmail.com](mailto:nickyeh0415@gmail.com)

