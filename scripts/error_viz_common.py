"""Shared RViz marker helpers for GICP-vs-GNSS error visualization."""

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def marker_point(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def error_color(cmap, err, color_max, alpha=1.0):
    level = float(np.clip(err / max(color_max, 1e-6), 0.0, 1.0))
    rgba = cmap(level)
    saturation = 0.18 + 0.82 * np.sqrt(level)
    base = np.array([0.22, 0.22, 0.24], dtype=float)
    rgb = base * (1.0 - saturation) + np.asarray(rgba[:3], dtype=float) * saturation
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))


def make_marker(frame, stamp, namespace, marker_id, marker_type, action=Marker.ADD):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = action
    return marker


def make_error_cylinders(frame, stamp, namespace, marker_id_start, base_xyz, top_xyz,
                         errors, diameter, alpha, color_fn):
    cylinders = []
    for i, (base_point, top_point, err) in enumerate(zip(base_xyz, top_xyz, errors)):
        height = float(top_point[2] - base_point[2])
        if height <= 1e-3:
            continue
        cylinder = make_marker(frame, stamp, namespace, marker_id_start + i, Marker.CYLINDER)
        cylinder.pose.position = marker_point(
            base_point[0], base_point[1], base_point[2] + height * 0.5)
        cylinder.pose.orientation.w = 1.0
        cylinder.scale.x = diameter
        cylinder.scale.y = diameter
        cylinder.scale.z = height
        cylinder.color = color_fn(err, alpha)
        cylinders.append(cylinder)
    return cylinders
