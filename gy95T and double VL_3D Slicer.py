"""
Paste this into Slicer's Python Interactor (View > Python Interactor),
AFTER you've connected to the combined GY-95T + dual-VL6180X Arduino sketch
via the ArduinoConnect module (Extension Manager > install
"ArduinoController" if you haven't already, then use the ArduinoConnect
module to connect at 115200 baud).

This script:
  1. Creates a cube model (if one doesn't already exist) representing the
     board.
  2. Creates a linear transform node (if one doesn't already exist) and
     parents the cube to it.
  3. Creates TWO "probe" tube models, one per VL6180X sensor, both mounted
     on the SAME face of the board (front edge, +Z), offset from each other
     so they don't overlap.
  4. Parses every field the combined Arduino sketch prints: acc_x/y
     (filtered, mm/s^2), acc_z, roll, pitch, yaw, temp, leve, mag_x/y/z,
     distance_mm (sensor1), distance_mm2 (sensor2). Raw/unfiltered fields
     are still parsed if present in the line but are not displayed.
  5. Rotates the cube+both probes live from roll/pitch/yaw.
  6. Scales each probe's length to its own live distance reading and colors
     it green/yellow/red depending on how close the sensed object is.
  7. Overlays filtered values and both distances as text in the 3D view's
     corner.
If your Arduino node isn't named "arduinoNode", change ARDUINO_NODE_NAME
below (check the Data module tree after connecting to see the actual name).
"""

import numpy as np
import vtk
import slicer

ARDUINO_NODE_NAME = "arduinoNode"      # <-- rename if yours differs
TRANSFORM_NODE_NAME = "GY95TTransform"
CUBE_NODE_NAME = "GY95TCube"
PROBE1_NODE_NAME = "VL6180XProbe1"
PROBE2_NODE_NAME = "VL6180XProbe2"

# --- Sensor mount geometry (in the cube's LOCAL coordinate frame, mm) ---
# ASSUMPTION: I don't know your physical layout, so these are placeholders.
# SENSOR*_LOCAL_POS is where the probe starts (offset from board center).
# SENSOR*_LOCAL_DIR is the direction it points (need not be unit length).
# Edit these to match how sensor1/sensor2 are actually mounted on your board.
# Both probes are now on the SAME face of the cube (front edge, pointing
# +Z), offset from each other along X so they sit side by side instead of
# overlapping.
SENSOR1_LOCAL_POS = (13, 67, 10)    # front edge, right of center
SENSOR1_LOCAL_DIR = (0.0, 1.0, 0.0)     # pointing +Y
SENSOR2_LOCAL_POS = (-13, 67, 10)   # front edge, left of center
SENSOR2_LOCAL_DIR = (0.0, 1.0, 0.0)     # pointing +Z (same side/direction)

# VL6180X default range is roughly 0-200mm; clamp the drawn probe so a
# stray/out-of-range reading doesn't stretch the line off into space.
PROBE_MAX_MM = 250.0
PROBE_RADIUS_MM = 1.5

# Proximity color thresholds (mm) for each probe.
PROBE_NEAR_MM = 50.0
PROBE_MID_MM = 150.0


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
        cubeSource.SetXLength(35)
        cubeSource.SetYLength(140)
        cubeSource.SetZLength(30)  # flattish, roughly board-shaped
        cubeSource.Update()

        node = slicer.modules.models.logic().AddModel(cubeSource.GetOutputPort())
        node.SetName(name)
        node.GetDisplayNode().SetColor(0.2, 0.6, 1.0)
        print(f"Created cube model '{name}'.")

    node.SetAndObserveTransformNodeID(transform_node.GetID())
    return node


def _build_probe_pipeline(local_pos, local_dir):
    """Builds a line+tube VTK pipeline from local_pos out along local_dir."""
    dir_arr = np.array(local_dir, dtype=float)
    norm = np.linalg.norm(dir_arr)
    unit_dir = dir_arr / norm if norm > 0 else np.array([0.0, 0.0, 1.0])

    lineSource = vtk.vtkLineSource()
    lineSource.SetPoint1(*local_pos)
    p2 = np.array(local_pos) + unit_dir * PROBE_RADIUS_MM  # placeholder length
    lineSource.SetPoint2(*p2)
    lineSource.Update()

    tubeFilter = vtk.vtkTubeFilter()
    tubeFilter.SetInputConnection(lineSource.GetOutputPort())
    tubeFilter.SetRadius(PROBE_RADIUS_MM)
    tubeFilter.SetNumberOfSides(12)
    tubeFilter.CappingOn()
    tubeFilter.Update()

    return lineSource, tubeFilter, tuple(local_pos), unit_dir


def get_or_create_probe(name, transform_node, local_pos, local_dir):
    """A tube from local_pos out along local_dir, length = live distance.

    Stored as attributes on the node so the update loop doesn't need module-
    level globals for the VTK source/filter objects.
    """
    node = slicer.mrmlScene.GetFirstNodeByName(name)
    if node is None:
        lineSource, tubeFilter, origin, unit_dir = _build_probe_pipeline(local_pos, local_dir)

        node = slicer.modules.models.logic().AddModel(tubeFilter.GetOutputPort())
        node.SetName(name)
        node.GetDisplayNode().SetColor(0.2, 1.0, 0.2)
        print(f"Created probe model '{name}'.")

        node._lineSource = lineSource
        node._tubeFilter = tubeFilter
        node._origin = origin
        node._unitDir = unit_dir
    else:
        # Node persisted from a previous run of this script in the same
        # session; VTK pipeline objects aren't persisted on the MRML node,
        # so python-side attributes would be gone too. Rebuild them.
        if not hasattr(node, "_lineSource"):
            lineSource, tubeFilter, origin, unit_dir = _build_probe_pipeline(local_pos, local_dir)
            node._lineSource = lineSource
            node._tubeFilter = tubeFilter
            node._origin = origin
            node._unitDir = unit_dir

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
        self.probe1Node = get_or_create_probe(
            PROBE1_NODE_NAME, self.transformNode, SENSOR1_LOCAL_POS, SENSOR1_LOCAL_DIR
        )
        self.probe2Node = get_or_create_probe(
            PROBE2_NODE_NAME, self.transformNode, SENSOR2_LOCAL_POS, SENSOR2_LOCAL_DIR
        )

        self.matrix = vtk.vtkMatrix4x4()

        # Center the 3D view on the cube once, so it's actually in frame.
        layoutManager = slicer.app.layoutManager()
        threeDWidget = layoutManager.threeDWidget(0)
        self.threeDView = threeDWidget.threeDView()
        threeDWidget.threeDController().resetFocalPoint()

        self.observerTag = self.arduinoNode.AddObserver(
            vtk.vtkCommand.ModifiedEvent, self.onArduinoDataUpdated
        )
        print("GY95TController is running. Move the board / wave a hand in front of either VL6180X.")

    def _updateProbe(self, probeNode, distance_mm):
        distance_mm = max(0.0, min(distance_mm, PROBE_MAX_MM))

        lineSource = probeNode._lineSource
        tubeFilter = probeNode._tubeFilter
        origin = np.array(probeNode._origin)
        unit_dir = probeNode._unitDir

        length = max(distance_mm, 0.01)  # avoid zero-length tube
        p2 = origin + unit_dir * length
        lineSource.SetPoint2(*p2)
        lineSource.Update()
        tubeFilter.Update()
        probeNode.SetAndObservePolyData(tubeFilter.GetOutput())

        if distance_mm < PROBE_NEAR_MM:
            color = (1.0, 0.2, 0.2)   # red: something close
        elif distance_mm < PROBE_MID_MM:
            color = (1.0, 0.85, 0.2)  # yellow: mid-range
        else:
            color = (0.2, 1.0, 0.2)   # green: clear / far
        probeNode.GetDisplayNode().SetColor(*color)

    def onArduinoDataUpdated(self, caller, event):
        raw = self.arduinoNode.GetParameter("Data")
        if not raw:
            return

        data = parse_line(raw)
        if not all(k in data for k in ("roll", "pitch", "yaw")):
            return  # incomplete line, skip

        # --- Update cube + both probes' orientation ---
        R = euler_to_rotation_matrix(data["roll"], data["pitch"], data["yaw"])
        for i in range(3):
            for j in range(3):
                self.matrix.SetElement(i, j, R[i, j])
        self.transformNode.SetMatrixTransformToParent(self.matrix)

        # --- Update each VL6180X range probe, if a reading was in this line ---
        if "distance_mm" in data:
            self._updateProbe(self.probe1Node, data["distance_mm"])
        if "distance_mm2" in data:
            self._updateProbe(self.probe2Node, data["distance_mm2"])

        # --- Update on-screen data overlay (top-left corner of 3D view) ---
        # Raw/unfiltered fields are intentionally omitted here.
        text = (
            f"Roll: {data.get('roll', 0):.2f}  "
            f"Pitch: {data.get('pitch', 0):.2f}  "
            f"Yaw: {data.get('yaw', 0):.2f}\n"
            f"Accel (mm/s^2)  X: {data.get('acc_x', 0):.0f}  "
            f"Y: {data.get('acc_y', 0):.0f}  "
            f"Z: {data.get('acc_z', 0):.0f}\n"
            f"Mag    X: {data.get('mag_x', 0):.0f}  "
            f"Y: {data.get('mag_y', 0):.0f}  "
            f"Z: {data.get('mag_z', 0):.0f}\n"
            f"Temp: {data.get('temp', 0):.2f} C   Level: {data.get('leve', 0):.0f}\n"
            f"Distance 1: {data.get('distance_mm', float('nan')):.0f} mm\n"
            f"Distance 2: {data.get('distance_mm2', float('nan')):.0f} mm"
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

