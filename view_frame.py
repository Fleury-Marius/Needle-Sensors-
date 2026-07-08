"""
Live viewer for ADNS3080 optical flow sensor frames.

Reads "FRAME_START" ... 900 pixel values (0-63) ... "FRAME_END"
from the Arduino serial output and displays it as a live grayscale image.

Install deps first:
    pip install pyserial matplotlib numpy

Usage:
    python view_frame.py COM5        (Windows)
    python view_frame.py /dev/ttyACM0   (Mac/Linux)
"""

import sys
import serial
import numpy as np
import matplotlib.pyplot as plt

GRID = 30
BAUD = 115200

def read_one_frame(ser):
    pixels = []
    # wait for FRAME_START
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "FRAME_START":
            break

    while len(pixels) < GRID * GRID:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "FRAME_END":
            break
        if line.isdigit():
            pixels.append(int(line))

    if len(pixels) != GRID * GRID:
        return None

    return np.array(pixels, dtype=np.uint8).reshape(GRID, GRID)


def main():
    if len(sys.argv) < 2:
        print("Usage: python view_frame.py <serial_port>")
        sys.exit(1)

    port = sys.argv[1]
    ser = serial.Serial(port, BAUD, timeout=2)

    plt.ion()
    fig, ax = plt.subplots()
    img = ax.imshow(np.zeros((GRID, GRID)), cmap="gray", vmin=0, vmax=63)
    ax.set_title("ADNS3080 live frame")
    plt.axis("off")

    print("Reading frames... close the plot window to stop.")
    try:
        while plt.fignum_exists(fig.number):
            frame = read_one_frame(ser)
            if frame is not None:
                img.set_data(frame)
                fig.canvas.draw()
                fig.canvas.flush_events()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
