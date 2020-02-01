import numpy
import cv2
from math import sin, cos, sqrt, atan2, inf, pi

class XDriveRenderer:
    def __init__(self, m_to_px, size = numpy.array([50, 50]), canvas_size = numpy.array([500, 500]), dt = 0.01):
        self.size = size
        self.m_to_px = m_to_px
        self.canvas_size = canvas_size
        self.dt = dt

    def render(self, img, X, X_dot):
        x, y, theta = X
        pos = numpy.array([x, -y])

        scaled_pos = pos * self.m_to_px
        center = self.canvas_size / 2
        # left_corner = center - self.size / 2 + scaled_pos
        # bottom_right_corner = center + self.size / 2 + scaled_pos
        
        v = sqrt(X_dot[0] ** 2 + X_dot[1] ** 2)
        v_theta = atan2(X_dot[1], X_dot[0])
        
        cv2.putText(img, f'x: {x:.2f}, y: {y:.2f}, theta: {(theta * 180 / pi):.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        cv2.putText(img, f'v: {v:.2f}, theta: {(v_theta * 180 / pi):.1f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        
        top_left_corner = numpy.array([
            center[0] + scaled_pos[0] + sin(-theta + pi / 4) * self.size[0],
            center[1] + scaled_pos[1] + cos(-theta + pi / 4) * self.size[1]
        ], dtype=numpy.int32)

        top_right_corner = numpy.array([
            center[0] + scaled_pos[0] + sin(-theta + pi * (3 / 4)) * self.size[0],
            center[1] + scaled_pos[1] + cos(-theta + pi * (3 / 4)) * self.size[1]
        ], dtype=numpy.int32)
        
        bottom_right_corner = numpy.array([
            center[0] + scaled_pos[0] + sin(-theta + pi * (5 / 4)) * self.size[0],
            center[1] + scaled_pos[1] + cos(-theta + pi * (5 / 4)) * self.size[1]
        ], dtype=numpy.int32)

        bottom_left_corner = numpy.array([
            center[0] + scaled_pos[0] + sin(-theta + pi * (7 / 4)) * self.size[0],
            center[1] + scaled_pos[1] + cos(-theta + pi * (7 / 4)) * self.size[1]
        ], dtype=numpy.int32)

        pts = numpy.array([top_left_corner, top_right_corner, bottom_right_corner, bottom_left_corner])

        cv2.polylines(img, numpy.int32([pts]), True, (255, 100, 0))


