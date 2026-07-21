#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 11:46:40 2026

@author: mariofleury
"""

"""
Paste this into Slicer's Python Interactor (View > Python Interactor),
AFTER you've connected to the GY-95T via the ArduinoConnect module
(Extension Manager > install "ArduinoController" if you haven't already,
then use the ArduinoConnect module to connect at 9600 baud).

This script:
  1. Creates a cube model (if one doesn't already exist).
  2. Creates a linear transform node (if one doesn't already exist) and
     parents the cube to it.
  3. Parses every field your Arduino sketch prints (acc_x/y/z, gyro_x/y/z,
     roll, pitch, yaw, temp, leve, mag_x/y/z).
  4. Rotates the cube live from roll/pitch/yaw.
  5. Overlays the raw accel + filtered gyro + mag values as text in the
     3D view's corner.

If your Arduino node isn't named "arduinoNode", change ARDUINO_NODE_NAME below
(check the Data module tree after connecting to see the actual name).
"""

import re
import numpy as np
import vtk
import slicer

ARDUINO_NODE_NAME = "arduinoNode"      # <-- rename if yours differs
TRANSFORM_NODE_NAME = "GY95TTransform"
CUBE_NODE_NAME = "GY95TCube"


def parse_line(raw):
    """Parses 'key: value, key: value, ...' lines into a dict of floats."""
    data = {}
    for token in raw.split(","):
        if ":" not in token:
            continue
        key, val = token.split(":", 1)
        key = key.strip()
        val = val.strip()
        try:
            data[key] = float(val)
        except ValueError:
            pass
    return data


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)],
    ])
    Ry = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)],
    ])
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1],
    ])
    return Rz @ Ry @ Rx


def get_or_create_transform(name):
    node = slicer.mrmlScene.GetFirstNodeByName(name)
    if node is None:
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", name)
        print(f"Created transform node '{name}'.")
    return node


def get_or_create_cube(name, transform_node):
    node = slicer.mrmlScene.GetFirstNodeByName(name)
    if node is None:
        cubeSource = vtk.vtkCubeSource()
        cubeSource.SetXLength(40)
        cubeSource.SetYLength(30)
        cubeSource.SetZLength(20)  # flattish, roughly board-shaped
        cubeSource.Update()

        node = slicer.modules.models.logic().AddModel(cubeSource.GetOutputPort())
        node.SetName(name)
        node.GetDisplayNode().SetColor(0.2, 0.6, 1.0)
        print(f"Created cube model '{name}'.")

    node.SetAndObserveTransformNodeID(transform_node.GetID())
    return node


class GY95TController:
    def __init__(self, arduino_node_name=ARDUINO_NODE_NAME):
        self.arduinoNode = slicer.mrmlScene.GetFirstNodeByName(arduino_node_name)
        if self.arduinoNode is None:
            raise RuntimeError(
                f"Could not find node '{arduino_node_name}'. "
                "Make sure ArduinoConnect is connected first."
            )

        self.transformNode = get_or_create_transform(TRANSFORM_NODE_NAME)
        self.cubeNode = get_or_create_cube(CUBE_NODE_NAME, self.transformNode)

        self.matrix = vtk.vtkMatrix4x4()

        # Center the 3D view on the cube once, so it's actually in frame.
        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        self.threeDView = threeDWidget.threeDView()
        threeDWidget.threeDController().resetFocalPoint()

        self.observerTag = self.arduinoNode.AddObserver(
            vtk.vtkCommand.ModifiedEvent, self.onArduinoDataUpdated
        )
        print("GY95TController is running. Move the IMU to see the cube move.")

    def onArduinoDataUpdated(self, caller, event):
        raw = self.arduinoNode.GetParameter("Data")
        if not raw:
            return

        data = parse_line(raw)
        if not all(k in data for k in ("roll", "pitch", "yaw")):
            return  # incomplete line, skip

        # --- Update cube orientation ---
        R = euler_to_rotation_matrix(data["roll"], data["pitch"], data["yaw"])
        for i in range(3):
            for j in range(3):
                self.matrix.SetElement(i, j, R[i, j])
        self.transformNode.SetMatrixTransformToParent(self.matrix)

        # --- Update on-screen data overlay (top-left corner of 3D view) ---
        # gyro_x/y/z here are the Kalman-filtered values printed by the
        # Arduino sketch (not gyro_x_raw/etc.) - same convention as
        # acc_x/acc_y below, which are also the filtered channels.
        text = (
            f"Roll: {data.get('roll', 0):.2f}  "
            f"Pitch: {data.get('pitch', 0):.2f}  "
            f"Yaw: {data.get('yaw', 0):.2f}\n"
            f"Accel  X: {data.get('acc_x', 0):.0f}  "
            f"Y: {data.get('acc_y', 0):.0f}  "
            f"Z: {data.get('acc_z', 0):.0f}\n"
            f"Gyro   X: {data.get('gyro_x', 0):.2f}  "
            f"Y: {data.get('gyro_y', 0):.2f}  "
            f"Z: {data.get('gyro_z', 0):.2f}\n"
            f"Mag    X: {data.get('mag_x', 0):.0f}  "
            f"Y: {data.get('mag_y', 0):.0f}  "
            f"Z: {data.get('mag_z', 0):.0f}\n"
            f"Temp: {data.get('temp', 0):.2f} C   Level: {data.get('leve', 0):.0f}"
        )
        self.threeDView.cornerAnnotation().SetText(2, text)  # 2 = upper-left
        self.threeDView.cornerAnnotation().GetTextProperty().SetColor(1, 1, 1)
        self.threeDView.forceRender()

    def stop(self):
        self.arduinoNode.RemoveObserver(self.observerTag)


# Instantiate to start live updates. Keep the reference (don't let it get
# garbage collected) or the observer stops firing.
controller = GY95TController()

# To stop later:
# controller.stop()
