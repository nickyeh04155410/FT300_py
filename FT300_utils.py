"""
ForceSensor Class for FT300 Force/Torque Sensor (Robotiq)

This class provides a convenient interface for communicating with the Robotiq FT300 force/torque sensor
via serial. It supports starting/stopping the data stream, activating streaming mode,
reading real-time force/torque data, setting a zero reference (offset), and performing CRC checks for message integrity.

Hardward preparation:
---------------------
The ft300 have to be connected to the PC via USB and power with a 24V power supply.

Note:
- Requires the 'pyserial', 'minimalmodbus', and 'libscrc' libraries.
- Serial parameters (COM port, baudrate, parity, etc.) should match your FT300 hardware configuration.

Author: Yu-Peng, Yeh, 2025-07-02, nickyeh0415@gmail.com
"""

import serial
import libscrc
import time
import minimalmodbus as mm

class ForceSensor:
    def __init__(self, port_name="COM5", slave_address=9, baudrate=19200, bytesize=8, parity="N", stopbits=1, timeout=1):
        """
        Initialize the ForceSensor object with serial and modbus parameters.
        """
        self.general_config = {
            "port": port_name,       # Serial port name (e.g., "COM5" or "/dev/ttyUSB0")
            "baudrate": baudrate,    # Baudrate for serial communication
            "bytesize": bytesize,    # Number of data bits
            "parity": parity,        # Parity bit ("N" for none)
            "stopbits": stopbits,    # Number of stop bits
            "timeout": timeout,      # Serial timeout in seconds
        }
        self.modbus_config = {"slave_address": slave_address}  # Modbus slave address
        self.ser = None  # Serial object, will be initialized on connection
        self.zeroRef = [0, 0, 0, 0, 0, 0]  # Zero reference values for force/torque compensation

    def stop_data_stream(self):
        """
        Stop the data stream from the FT300 sensor by sending a specific byte sequence.
        """
        ser = serial.Serial(**self.general_config)
        packet = bytearray([0xff] * 50) # 50 bytes of 0xFF to stop streaming
        ser.write(packet)
        ser.close()

    def activate_streaming(self):
        """
        Activate streaming mode on the FT300 using Modbus command.
        """
        # Set modbus parameters globally for minimalmodbus        
        mm.BAUDRATE = self.general_config["baudrate"]
        mm.BYTESIZE = self.general_config["bytesize"]
        mm.PARITY = self.general_config["parity"]
        mm.STOPBITS = self.general_config["stopbits"]
        mm.TIMEOUT = self.general_config["timeout"]

        # Open modbus instrument and send command to start streaming
        ft300 = mm.Instrument(self.general_config["port"], slaveaddress=self.modbus_config["slave_address"])
        ft300.close_port_after_each_call = True
        ft300.write_register(410, 0x0200) # Write 0x0200 to register 410 to activate streaming
        del ft300

    def set_zero_reference(self):
        """
        Set the zero reference for force/torque measurements.
        This function reads two packets from serial and uses the second as the new zero.
        """
        STARTBYTES = bytes([0x20, 0x4e])

        # Read messages from serial until the start marker is found
        data = self.ser.read_until(STARTBYTES)
        data = self.ser.read_until(STARTBYTES)
        dataArray = bytearray(data)
        dataArray = STARTBYTES + dataArray[:-2]
        
        if not self.crcCheck(dataArray):
            raise ValueError("CRC mismatch when setting zero reference.")
        
        # Extract force/torque values as zero reference
        self.zeroRef = self.forceFromSerialMessage(dataArray, zeroRef=[0, 0, 0, 0, 0, 0])


    def serial_connect(self):
        """
        Establish serial connection using the given configuration.
        """
        self.ser = serial.Serial(**self.general_config)
        return self.ser

    def read_data(self, init=True):
        """
        Read a full force/torque data packet from the sensor.
        If 'init' is True, discard the first incomplete message and use the second.
        """
        STARTBYTES = bytes([0x20, 0x4e])
        if init == True:
            # Read serial buffer until finding the bytes [0x20,0x4e]
            # First serial reading.
            # This message is incomplete in most cases, so it is ignored.
            data = self.ser.read_until(STARTBYTES)

            # Second serial reading.
            # This message is used to make the zero of the sensor.
            data = self.ser.read_until(STARTBYTES)
            # Convert from byte to bytearray
            dataArray = bytearray(data)
            # Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
            dataArray = STARTBYTES + dataArray[:-2]
        else:
            data = self.ser.read_until(STARTBYTES)
            # Convert from byte to bytearray
            dataArray = bytearray(data)
            # Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
            dataArray = STARTBYTES + dataArray[:-2]

        return dataArray

    def forceFromSerialMessage(self, serialMessage, zeroRef=None):
        """
        Convert a serial message to force/torque values, with optional zero reference compensation.

        Args:
            serialMessage (bytearray): The raw message from the sensor.
            zeroRef (list): The zero reference to subtract (default is self.zeroRef).

        Returns:
            forceTorque (list): List of 6 compensated force/torque values [Fx, Fy, Fz, Tx, Ty, Tz].
        """
        forceTorque=[0,0,0,0,0,0]

        if zeroRef is None:
            zeroRef = self.zeroRef
        
        # Extract and scale sensor data (Fx, Fy, Fz in N; Tx, Ty, Tz in Nm)
        forceTorque = [
            round(int.from_bytes(serialMessage[2:4], byteorder='little', signed=True)/100 - zeroRef[0], 2),
            round(int.from_bytes(serialMessage[4:6], byteorder='little', signed=True)/100 - zeroRef[1], 2),
            round(int.from_bytes(serialMessage[6:8], byteorder='little', signed=True)/100 - zeroRef[2], 2),
            round(int.from_bytes(serialMessage[8:10], byteorder='little', signed=True)/1000 - zeroRef[3], 2),
            round(int.from_bytes(serialMessage[10:12], byteorder='little', signed=True)/1000 - zeroRef[4], 2),
            round(int.from_bytes(serialMessage[12:14], byteorder='little', signed=True)/1000 - zeroRef[5], 2),
        ]
        return forceTorque

    def crcCheck(self, serialMessage):
        """
        Verify CRC for the received message.

        Args:
            serialMessage (bytearray): Message to check.

        Returns:
            bool: True if CRC matches, False otherwise.
        """
        crc = int.from_bytes(serialMessage[14:16], byteorder='little', signed=False)    # Extract CRC from message
        crcCalc = libscrc.modbus(serialMessage[0:14])   # Compute CRC of message data
        return crc == crcCalc

    def read_sensor(self, FT_data, lock, use_zero_ref=True):
        """
        Start and continuously read data from the FT300 force/torque sensor, updating a shared memory array.

        Args:
            FT_data (multiprocessing.Array): Shared array to store 6D sensor data [Fx, Fy, Fz, Tx, Ty, Tz].
            lock (multiprocessing.Lock): Lock for synchronizing access to shared data.
            use_zero_ref (bool): Whether to perform zero reference (tare) at start (default: True).

        This method runs an infinite loop, reading sensor values and writing them to FT_data in a thread/process-safe way.
        """
        self.stop_data_stream()      # Stop any previous data stream
        self.activate_streaming()    # Activate streaming mode
        self.serial_connect()        # Open serial connection

        # Optionally perform zero reference (tare)
        if use_zero_ref:
            self.set_zero_reference()

        nbrMessages = 0
        startTime = time.time()

        while True:
            # Read raw data packet from sensor
            dataArray = self.read_data(init=False)
            if self.crcCheck(dataArray):
                # Decode force/torque values (Fx, Fy, Fz, Tx, Ty, Tz)
                forceTorque_ee = self.forceFromSerialMessage(dataArray)
                # Write values into shared array (with lock to prevent race conditions)
                with lock:
                    for i in range(len(forceTorque_ee)):
                        FT_data[i] = forceTorque_ee[i]

                # (Optional) Calculate and print reading frequency for monitoring
                nbrMessages += 1
                elapsedTime = time.time() - startTime
                if elapsedTime > 0:
                    frequency = round(nbrMessages / elapsedTime)
                    # print(f"Sensor reading frequency: {frequency} Hz")
                else:
                    frequency = 0

# --- Template for using multiprocessing to read data from the FT300 force/torque sensor ---
# --- Do not uncomment ---
# if __name__ == "__main__":
#     from multiprocessing import Array, Lock, Process
    
#     FT_data = Array('d', [0]*6)  # 用來儲存 [Fx, Fy, Fz, Tx, Ty, Tz]
#     lock = Lock()

#     # 建立感測器物件
#     sensor = ForceSensor(port_name="COM5")  # port_name 請依你實際狀況修改

#     # 啟動感測器讀取資料的子程序
#     p = Process(target=sensor.read_sensor, args=(FT_data, lock), kwargs={"use_zero_ref": True})  # If you do not want to use zero reference, set use_zero_ref=False
#     p.daemon = True  # 主程式結束時自動關閉子程序
#     p.start()

#     # 主程式可定期讀取最新資料
#     try:
#         PERIOD = 0.01  # 期望週期 (秒)，100Hz
#         next_time = time.perf_counter()
#         count = 0
#         last_print = time.perf_counter()
#         while True:
#             t0 = time.time()
            
#             with lock:
#                 ft_values = [FT_data[i] for i in range(6)]
#             print(f"Current FT300 readings: Fx={ft_values[0]:.2f}N, Fy={ft_values[1]:.2f}N, Fz={ft_values[2]:.2f}N, "
#                   f"Tx={ft_values[3]:.3f}Nm, Ty={ft_values[4]:.3f}Nm, Tz={ft_values[5]:.3f}Nm")
        
#             count += 1
#             # --------- 每秒顯示一次 Hz ---------
#             now = time.perf_counter()
#             if now - last_print >= 1.0:
#                 print(f"Control loop frequency: {count} Hz")
#                 count = 0
#                 last_print += 1.0  # 防止累積飄移（比設 last_print = now 更精準）

#             # --------- 精準等週期控制 ---------
#             next_time += PERIOD
#             sleep_time = next_time - time.perf_counter()
#             if sleep_time > 0:
#                 if sleep_time > 0.002:
#                     # 若時間足夠，先 sleep 粗略等
#                     time.sleep(sleep_time - 0.001)
#                 # 最後 1ms 用 busy-wait 微調
#                 while time.perf_counter() < next_time:
#                     pass
#             else:
#                 # 若超時，重設 next_time 避免累積誤差
#                 next_time = time.perf_counter()

#     except KeyboardInterrupt:
#         print("Exiting...")  # 按 Ctrl+C 可終止主程式與感測子程序