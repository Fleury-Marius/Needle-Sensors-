#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Load the phantom pegboard twin into Slicer and drop a labeled marker at
every LABELED hole (A1..G7), at its true physical coordinate.

Frame (must match your needle tracking):
  origin (0,0,0) = hole D4, on the FRONT face of the board (Z=0)
  +X = A -> G (right)      +Y = 1 -> 7 (up)      +Z = out toward needle

Usage in Slicer's Python Interactor:
  1. Load phantom_board.stl (File > Add Data), or set PHANTOM_STL_PATH.
  2. Paste this script.
It creates:
  - a markups fiducial node with the 49 labeled targets (A1..G7)
  - optional finer markers at every 5 mm grid hole (SHOW_ALL_HOLES)
So when you drive the tracked needle to a hole, both the target marker
and the live needle tip are visible and the gap between them is your
tracking error, readable straight off the scene.

Measuring error numerically:
  controller_target("D4")   # prints distance from the live needle tip
                            # to hole D4, updated as you move
(Only works if your GY95TController from the tracking script is running
in the same interpreter, so the needle tip position is available.)
"""

import numpy as np
import slicer

PHANTOM_STL_PATH = None          # set to load automatically if not in scene
PHANTOM_NODE_NAME = "phantom_pegboard"

TARGETS_NODE_NAME = "PhantomTargets"
ALLHOLES_NODE_NAME = "PhantomAllHoles"

PITCH = 5.0                      # mm, fine grid
LABEL_STEP = 10.0                # mm between labeled lines
COLS = "ABCDEFG"                 # labeled columns (X)
ROWS = [1, 2, 3, 4, 5, 6, 7]     # labeled rows (Y)

SHOW_ALL_HOLES = False           # also mark every 5 mm hole (unlabeled)
FRONT_Z = 0.0                    # markers sit on the front face


def labeled_coord(col, row):
    """World coordinate (mm) of a labeled hole, D4 at origin."""
    ci = COLS.index(col)                 # 0..6, D=3
    x = (ci - COLS.index("D")) * LABEL_STEP
    y = (row - 4) * LABEL_STEP
    return (x, y, FRONT_Z)


def ensure_phantom_loaded():
    node = slicer.mrmlScene.GetFirstNodeByName(PHANTOM_NODE_NAME)
    if node is None and PHANTOM_STL_PATH:
        node = slicer.util.loadModel(PHANTOM_STL_PATH)
        if node is not None:
            node.SetName(PHANTOM_NODE_NAME)
    if node is not None:
        d = node.GetDisplayNode()
        if d:
            d.SetColor(0.25, 0.25, 0.28)
            d.SetOpacity(0.85)
    return node


def make_target_markers():
    existing = slicer.mrmlScene.GetFirstNodeByName(TARGETS_NODE_NAME)
    if existing is not None:
        slicer.mrmlScene.RemoveNode(existing)

    fid = slicer.mrmlScene.AddNewNodeByClass(
        "vtkMRMLMarkupsFiducialNode", TARGETS_NODE_NAME)
    for row in reversed(ROWS):           # 7 at top
        for col in COLS:
            x, y, z = labeled_coord(col, row)
            idx = fid.AddControlPoint(x, y, z)
            fid.SetNthControlPointLabel(idx, f"{col}{row}")

    d = fid.GetDisplayNode()
    d.SetGlyphScale(2.0)
    d.SetTextScale(2.5)
    d.SetSelectedColor(1.0, 0.85, 0.1)
    d.SetColor(1.0, 0.85, 0.1)

    # Lock every control point AND the node itself so nothing can be
    # dragged, nudged, or deleted by a stray click. Their coordinates are
    # ground truth; a moved marker is a wrong measurement. Belt and
    # braces here because the two APIs differ across Slicer versions.
    fid.SetLocked(True)
    fid.LockedOn()
    for i in range(fid.GetNumberOfControlPoints()):
        fid.SetNthControlPointLocked(i, True)

    print(f"Placed {fid.GetNumberOfControlPoints()} labeled targets "
          "(A1..G7), locked.")
    return fid


def make_all_hole_markers():
    if not SHOW_ALL_HOLES:
        return None
    existing = slicer.mrmlScene.GetFirstNodeByName(ALLHOLES_NODE_NAME)
    if existing is not None:
        slicer.mrmlScene.RemoveNode(existing)

    fid = slicer.mrmlScene.AddNewNodeByClass(
        "vtkMRMLMarkupsFiducialNode", ALLHOLES_NODE_NAME)
    n = int(round(60.0 / PITCH)) + 1     # 13
    for j in range(n):
        for i in range(n):
            x = -30.0 + i * PITCH
            y = -30.0 + j * PITCH
            idx = fid.AddControlPoint(x, y, FRONT_Z)
            fid.SetNthControlPointLabel(idx, "")   # unlabeled
    d = fid.GetDisplayNode()
    d.SetGlyphScale(0.8)
    d.SetTextScale(0.0)
    d.SetSelectedColor(0.4, 0.7, 1.0)
    d.SetColor(0.4, 0.7, 1.0)

    fid.SetLocked(True)
    fid.LockedOn()
    for i in range(fid.GetNumberOfControlPoints()):
        fid.SetNthControlPointLocked(i, True)

    print(f"Placed {fid.GetNumberOfControlPoints()} fine grid markers, locked.")
    return fid


# --- error readout helper -------------------------------------------------
def controller_target(name, controller_var="controller"):
    """Print the distance from the live needle tip to a labeled hole.

    Needs the tracking controller running in this interpreter. The needle
    tip world position is recovered from its transform the same way the
    trail does."""
    col, row = name[0].upper(), int(name[1:])
    tx, ty, tz = labeled_coord(col, row)
    target = np.array([tx, ty, tz])

    ctrl = globals().get(controller_var) or __builtins__.__dict__.get(controller_var)
    if ctrl is None:
        print(f"No '{controller_var}' found. Is the tracking script running?")
        return

    # Recompute tip world position from the controller's current pose.
    R = np.array([[ctrl.matrix.GetElement(i, j) for j in range(3)] for i in range(3)])
    t = np.array([ctrl.matrix.GetElement(i, 3) for i in range(3)])
    tipWorld = R @ (ctrl.tipLocal) + t
    err = tipWorld - target
    dist = float(np.linalg.norm(err))
    print(f"Target {name} at {tuple(target)}")
    print(f"Needle tip at ({tipWorld[0]:.1f}, {tipWorld[1]:.1f}, {tipWorld[2]:.1f})")
    print(f"Error: {dist:.1f} mm   (dx={err[0]:+.1f} dy={err[1]:+.1f} dz={err[2]:+.1f})")
    return dist


ensure_phantom_loaded()
make_target_markers()
make_all_hole_markers()
print("Phantom targets ready. Zero your needle at D4 (origin), then use "
      "controller_target('D4') etc. to read tip-to-hole error live.")