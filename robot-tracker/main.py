from renderer import XDriveRenderer
from math import sqrt, pi
import numpy
import matplotlib.pyplot as plt
import cv2
import atexit, io, sys

# output_buffer = io.BytesIO()
# sys.stdout = output_buffer

# @atexit.register 
# def write(): 
#     sys.__stdout__.write(output_buffer.getvalue()) 

canvas_size = numpy.array([800, 800])

m_to_px = 130
f_to_px = 130 * 0.3048
dt = 0.05 # 10ms

drive_renderer = XDriveRenderer(f_to_px, numpy.array([30, 30]), canvas_size, dt)

img = numpy.zeros((canvas_size[0], canvas_size[1], 3), numpy.uint8)

center = canvas_size / 2
last_point = (int(center[0]), int(center[1]))
n = 0

while True:
  data = input()
  dataList = data.split(',')
  if len(dataList) != 6:
    print("Got Invalid Data, Exiting")
    break
  # print("ran " + str(n) + " times")
  n = n + 1
  x = numpy.array([float(dataList[0]), float(dataList[1]), float(dataList[2])])
  x_dot = numpy.array([float(dataList[3]), float(dataList[4]), float(dataList[5])])

  # plot robot path

  # cv2.imshow('2150A Robot Plotter', img)
  pos = numpy.array([x[0], -x[1]])
  scaled_pos = pos * f_to_px
  scaled_pos_int = numpy.array([scaled_pos[0] + center[0], scaled_pos[1] + center[1]], dtype=numpy.int32)
  # plot robot
  if last_point != (scaled_pos_int[0], scaled_pos_int[1]):
    cv2.line(img, (scaled_pos_int[0], scaled_pos_int[1]), last_point, (0, 255, 0), 2)
    print("Line at: {}, {}, {}, {}".format(scaled_pos_int[0], scaled_pos_int[1], last_point[0], last_point[1]))
  last_point = (scaled_pos_int[0], scaled_pos_int[1])

  tmp = img.copy()

  drive_renderer.render(tmp, x, x_dot)

  cv2.imshow('2150A Robot Plotter', tmp)
  cv2.waitKey(int(1000 * dt))
