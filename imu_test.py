import serial
import struct
import time

SERIAL_PORT = '/dev/cu.usbserial-1110'  
BAUD_RATE = 115200  

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
                        # Shifted indexing to accurately capture the stable Euler registers
                        yaw   = struct.unpack('>h', packet[13:15])[0] / 100.0
                        pitch = struct.unpack('>h', packet[15:17])[0] / 100.0
                        roll  = struct.unpack('>h', packet[17:19])[0] / 100.0
                        
                        # Clean up formatting limits to display intuitive -180 to +180 ranges
                        if yaw > 180: yaw -= 360
                        if pitch > 180: pitch -= 360
                        if roll > 180: roll -= 360
                        
                        print(f"Yaw: {yaw:7.2f}° | Pitch: {pitch:7.2f}° | Roll: {roll:7.2f}°")
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

