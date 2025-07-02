"""
FT300MonitorUI – Real-Time Force/Torque Visualization and Logging for Robotiq FT300

Overview:
---------
FT300MonitorUI provides an intuitive GUI for live monitoring and CSV data logging
of Robotiq FT300 6-axis force/torque sensors. The tool leverages a separate acquisition
process for robust performance, ensuring a responsive UI even during high-frequency (100 Hz) streaming.

Requirements:
-------------
- numpy
- pyqtgraph
- PyQt6
- pyserial
- FT300_utils.py (provides ForceSensor class)
- Python 3.8

Author: Yu-Peng Yeh, 2025-07-02, nickyeh0415@gmail.com
"""

import sys
import serial.tools.list_ports
import numpy as np
import pyqtgraph as pg
from FT300_utils import ForceSensor
from PyQt6 import QtWidgets, QtCore
from multiprocessing import Array, Lock, Process, freeze_support

def run_sensor_proc(port, FT_data, lock):
    sensor = ForceSensor(port_name=port)
    sensor.read_sensor(FT_data, lock)

class FT300MonitorUI(QtWidgets.QWidget):
    """
    UI for real-time FT300 force/torque monitoring, with drop-down COM port selection and Start/Stop control.
    Curves are hidden when not acquiring.
    """
    def __init__(self, ft_data, lock):
        super().__init__()
        self.setWindowTitle("FT300 Force/Torque Sensor UI (PyQtGraph)")
        self.resize(900, 600)
        self.ft_data = ft_data
        self.lock = lock

        self.data_len = 200
        self.data = np.zeros((self.data_len, 6))
        self.is_saving = False
        self.save_path = None
        self.recorded_data = []
        self.sensor_proc = None  # For acquisition process
        self.acquiring = False   # Track if process is running

        # --- UI Layout ---
        layout = QtWidgets.QVBoxLayout(self)
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # Force plot
        self.force_plot = self.plot_widget.addPlot(row=0, col=0, title="Force [N]")
        self.force_plot.showGrid(x=True, y=True)
        legend = self.force_plot.addLegend(offset=(5, 25), colCount=3)
        legend.anchor((0,0), (0,0))
        force_colors = [(200,80,80), (80,200,80), (80,120,255)]
        self.force_curves = []
        for i, lbl in enumerate(['Fx','Fy','Fz']):
            curve = self.force_plot.plot(
                pen=pg.mkPen(color=force_colors[i], width=2), name=lbl)
            self.force_curves.append(curve)
        self.force_plot.setYRange(-20, 20)
        self.force_plot.setLabel('left', 'Force (N)')

        # Torque plot
        self.torque_plot = self.plot_widget.addPlot(row=1, col=0, title="Torque [Nm]")
        self.torque_plot.showGrid(x=True, y=True)
        legend = self.torque_plot.addLegend(offset=(5, 25), colCount=3)
        legend.anchor((0,0), (0,0))
        torque_colors = [(200,80,80), (80,200,80), (80,120,255)]
        self.torque_curves = []
        for i, lbl in enumerate(['Tx','Ty','Tz']):
            curve = self.torque_plot.plot(
                pen=pg.mkPen(color=torque_colors[i], width=2), name=lbl)
            self.torque_curves.append(curve)
        self.torque_plot.setYRange(-2, 2)
        self.torque_plot.setLabel('left', 'Torque (Nm)')

        # --- Controls ---
        hbox = QtWidgets.QHBoxLayout()
        layout.addLayout(hbox)

        # COM port selector (ComboBox)
        hbox.addWidget(QtWidgets.QLabel("COM port:"))
        self.combox = QtWidgets.QComboBox()
        self.refresh_com_ports()
        hbox.addWidget(self.combox)
        self.refresh_btn = QtWidgets.QPushButton("↻")
        self.refresh_btn.setFixedWidth(36)
        self.refresh_btn.clicked.connect(self.refresh_com_ports)
        hbox.addWidget(self.refresh_btn)

        # Start/Stop button for acquisition
        self.start_btn = QtWidgets.QPushButton("Start")
        self.start_btn.setFixedHeight(36)
        self.start_btn.setStyleSheet("font-size: 16px;")
        self.start_btn.clicked.connect(self.toggle_sensor_process)
        hbox.addWidget(self.start_btn)

        # Save controls
        hbox.addWidget(QtWidgets.QLabel("Save path:"))
        self.path_label = QtWidgets.QLabel("(not set)")
        self.path_label.setMinimumWidth(320)
        hbox.addWidget(self.path_label)
        self.browse_btn = QtWidgets.QPushButton("Browse...")
        self.browse_btn.setFixedHeight(36)
        self.browse_btn.setStyleSheet("font-size: 16px;")
        self.browse_btn.clicked.connect(self.choose_path)
        hbox.addWidget(self.browse_btn)
        self.save_btn = QtWidgets.QPushButton("Save")
        self.save_btn.setFixedHeight(36)
        self.save_btn.setStyleSheet("font-size: 16px;")
        self.save_btn.clicked.connect(self.toggle_save)
        hbox.addWidget(self.save_btn)

        # Y-axis scaling buttons
        self.fix_y_btn = QtWidgets.QPushButton("Fixed Y-Axis")
        self.fix_y_btn.setFixedHeight(36)
        self.fix_y_btn.setStyleSheet("font-size: 16px;")
        self.fix_y_btn.clicked.connect(self.set_fixed_yrange)
        hbox.addWidget(self.fix_y_btn)
        self.auto_y_btn = QtWidgets.QPushButton("Auto Y-Axis")
        self.auto_y_btn.setFixedHeight(36)
        self.auto_y_btn.setStyleSheet("font-size: 16px;")
        self.auto_y_btn.clicked.connect(self.set_auto_yrange)
        hbox.addWidget(self.auto_y_btn)

        # Timer for UI updates (100 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)  # 10 ms = 100Hz

    def refresh_com_ports(self):
        """Refresh available serial ports in the ComboBox."""
        self.combox.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if not ports:
            self.combox.addItem("No Port")
            self.combox.setDisabled(True)
        else:
            self.combox.addItems(ports)
            self.combox.setDisabled(False)

    def toggle_sensor_process(self):
        """Start or stop the sensor acquisition process."""
        if self.sensor_proc is None or not self.sensor_proc.is_alive():
            port = self.combox.currentText()
            if "No Port" in port:
                QtWidgets.QMessageBox.warning(self, "No Port", "No COM port detected!")
                return
            self.sensor_proc = Process(target=run_sensor_proc, args=(port, self.ft_data, self.lock))
            self.sensor_proc.daemon = True
            self.sensor_proc.start()
            self.start_btn.setText("Stop")
            self.combox.setDisabled(True)
            self.refresh_btn.setDisabled(True)
            self.acquiring = True
        else:
            self.sensor_proc.terminate()
            self.sensor_proc.join()
            self.sensor_proc = None
            self.start_btn.setText("Start")
            self.combox.setDisabled(False)
            self.refresh_btn.setDisabled(False)
            self.acquiring = False
            self.data = np.zeros((self.data_len, 6))  # Reset data to blank

    def choose_path(self):
        """Open a file dialog to select CSV save location."""
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Data", filter="CSV Files (*.csv)")
        if filename:
            self.save_path = filename
            self.path_label.setText(filename)

    def toggle_save(self):
        """Start or stop data recording to CSV."""
        if not self.is_saving:
            self.is_saving = True
            self.save_btn.setText("Stop Save")
            self.recorded_data = []
        else:
            self.is_saving = False
            self.save_btn.setText("Save")
            self.save_data()

    def save_data(self):
        """Save recorded data to CSV file."""
        if self.save_path and self.recorded_data:
            import csv
            with open(self.save_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
                writer.writerows(self.recorded_data)
            QtWidgets.QMessageBox.information(self, "Saved", f"Saved {len(self.recorded_data)} records to\n{self.save_path}")
        else:
            QtWidgets.QMessageBox.warning(self, "No Data", "No data to save or path not set.")

    def set_fixed_yrange(self):
        """Set force and torque plots to fixed Y-axis range."""
        self.force_plot.setYRange(-20, 20)
        self.torque_plot.setYRange(-2, 2)

    def set_auto_yrange(self):
        """Enable auto-scaling for force and torque Y-axis."""
        self.force_plot.enableAutoRange('y', True)
        self.torque_plot.enableAutoRange('y', True)

    def update_plot(self):
        """Update plots with new data from the shared array. Hide curves if not acquiring."""
        if self.acquiring:
            with self.lock:
                latest = [float(self.ft_data[i]) for i in range(6)]
            self.data = np.vstack([self.data[1:], np.array(latest).reshape(1, 6)])
            if self.is_saving:
                self.recorded_data.append(list(self.data[-1]))
            # Show curves
            for i in range(3):
                self.force_curves[i].setData(self.data[:, i])
                self.force_curves[i].setVisible(True)
                self.torque_curves[i].setData(self.data[:, i+3])
                self.torque_curves[i].setVisible(True)
        else:
            # Hide curves when not acquiring
            for i in range(3):
                self.force_curves[i].setVisible(False)
                self.torque_curves[i].setVisible(False)

    def closeEvent(self, event):
        """Ensure acquisition process is terminated when UI closes."""
        if self.sensor_proc and self.sensor_proc.is_alive():
            self.sensor_proc.terminate()
            self.sensor_proc.join()
        event.accept()

if __name__ == "__main__":
    freeze_support()
    FT_data = Array('d', [0]*6)
    lock = Lock()
    app = QtWidgets.QApplication(sys.argv)
    win = FT300MonitorUI(FT_data, lock)
    win.show()
    sys.exit(app.exec())
