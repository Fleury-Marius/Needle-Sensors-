
""" Required extension on 3D slicer: SlicerIGT and OpenIGTLink """
'''On 3D-Slicer, make sure, when connected to the Module OpenIGTLinkIF, 
that the scene is set to be server and Active with Port number 18944 
(those can be changed in properties) '''
'''Before running make sure you have changed your SERIAL_PORT for the one in 
your computer'''

'''Adapted code from imu_test.py to be visualized on 3d-Slicer'''

import serial
import struct
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import pyigtl

SERIAL_PORT = '/dev/cu.usbserial-1110'
BAUD_RATE = 115200

# Set up the bridge to 3D Slicer (Matches the port in Slicer)
print("Connecting to 3D Slicer OpenIGTLink Server...")
igtl_client = pyigtl.OpenIGTLinkClient(host="127.0.0.1", port=18944)

def parse_imu_calibrated():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected on {SERIAL_PORT}. Reading CALIBRATED data stream...")
        time.sleep(0.5)

        while True:
            if ser.read(1) == b'\x5A':
                packet = ser.read(23)
                if len(packet) == 23:
                    try:
                        # Parsing Euler registers
                        yaw = struct.unpack('>h', packet[13:15])[0] / 100.0
                        pitch = struct.unpack('>h', packet[15:17])[0] / 100.0
                        roll = struct.unpack('>h', packet[17:19])[0] / 100.0

                        if yaw > 180: yaw -= 360
                        if pitch > 180: pitch -= 360
                        if roll > 180: roll -= 360

                        print(f"Yaw: {yaw:7.2f}° | Pitch: {pitch:7.2f}° | Roll: {roll:7.2f}°")

                        # 1. Convert Euler angles to a 4x4 3D Rotation Matrix
                        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
                        transform_matrix = np.eye(4)
                        transform_matrix[:3, :3] = rot.as_matrix()

                        # 2. Package it and send it to Slicer
                        transform_message = pyigtl.TransformMessage(transform_matrix, device_name="IMU_Rotation")
                        igtl_client.send_message(transform_message)

                    except Exception as e:
                        print(f"Parsing error: {e}")

    except serial.SerialException as e:
        print(f"Connection Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping IMU data reader.")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    parse_imu_calibrated()
    