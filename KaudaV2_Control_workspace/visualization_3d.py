"""
3D Visualization utilities for KaudaV2 Control
"""
import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLLinePlotItem, GLTextItem


def make_cylinder(radius, length, slices=32):
    """
    Create a cylindrical MeshData aligned along local +X (height = length along X).
    We create two rings of vertices (start and end) and faces for the sides.
    
    Args:
        radius: Cylinder radius
        length: Cylinder length along X axis
        slices: Number of slices around the cylinder
    Returns:
        MeshData: Cylinder mesh
    """
    theta = np.linspace(0, 2*np.pi, slices, endpoint=False)
    y = radius * np.cos(theta)
    z = radius * np.sin(theta)
    x0 = np.zeros(slices, dtype=np.float32)
    x1 = np.ones(slices, dtype=np.float32) * float(length)

    verts0 = np.column_stack((x0, y, z))
    verts1 = np.column_stack((x1, y, z))
    verts = np.vstack((verts0, verts1)).astype(np.float32)

    faces = []
    n = slices
    for i in range(n):
        ni = (i + 1) % n
        # quad -> two triangles
        faces.append([i, ni, n + ni])
        faces.append([i, n + ni, n + i])
    faces = np.array(faces, dtype=np.int32)
    md = MeshData(vertexes=verts, faces=faces)
    return md


def make_box(dx, dy, dz):
    """
    Create a box MeshData
    
    Args:
        dx, dy, dz: Box dimensions
    Returns:
        MeshData: Box mesh
    """
    md = MeshData.cube()
    # cube vertices are [-1,1] unit cube -> we scale
    md = MeshData(vertexes=(md.vertexes() * np.array([dx/2.0, dy/2.0, dz/2.0])))
    return md


def draw_local_frame(view, frame, name="", length=20, 
                     color_x=(1, 0, 0, 1), color_y=(0, 1, 0, 1), color_z=(0, 0, 1, 1)):
    """
    Draw the X, Y, Z axes of a local frame and add a visible label.
    
    Args:
        view: GLViewWidget to add items to
        frame: 4x4 transformation matrix
        name: Frame name for label
        length: Length of axes
        color_x, color_y, color_z: Colors for X, Y, Z axes
    Returns:
        list: List of GLItems added to the view
    """
    origin = frame[:3, 3]
    items = []

    # X axis (red)
    x_axis_end = origin + frame[:3, :3] @ np.array([length, 0, 0])
    x_axis = np.array([origin, x_axis_end])
    x_line = GLLinePlotItem(pos=x_axis, color=color_x, width=2, antialias=True)
    view.addItem(x_line)
    items.append(x_line)

    # Y axis (green)
    y_axis_end = origin + frame[:3, :3] @ np.array([0, length, 0])
    y_axis = np.array([origin, y_axis_end])
    y_line = GLLinePlotItem(pos=y_axis, color=color_y, width=2, antialias=True)
    view.addItem(y_line)
    items.append(y_line)

    # Z axis (blue)
    z_axis_end = origin + frame[:3, :3] @ np.array([0, 0, length])
    z_axis = np.array([origin, z_axis_end])
    z_line = GLLinePlotItem(pos=z_axis, color=color_z, width=2, antialias=True)
    view.addItem(z_line)
    items.append(z_line)

    # Add label for the frame
    if name:
        # Offset to avoid overlap with axes
        text_offset = frame[:3, :3] @ np.array([0, 0, length * 1.2])  # Offset along local Z
        text_pos = origin + text_offset

        # Create text
        text = GLTextItem(pos=text_pos, text=name, color=(1, 1, 1, 1))

        # Add to view
        view.addItem(text)
        items.append(text)

    return items
