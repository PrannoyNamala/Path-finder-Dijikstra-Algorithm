#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 28 19:12:03 2021

@author: prannoy
"""
import cv2
import numpy as np
import math


def point_2(pt, distance, angle):
    _angle = angle * math.pi / 180.0
    cos = math.cos(_angle)
    sin = math.sin(_angle)
    return int(pt[0] + (distance * cos)), int(pt[1] - (distance * sin))


def create_main_map():
    main_canvas = np.zeros((300, 400), np.uint8)

    # Circle
    main_canvas = cv2.circle(main_canvas, (90, 300 - 70), 35, 255, -1)

    # Ellipse
    main_canvas = cv2.ellipse(main_canvas, (246, 300 - 175), (60, 30), 0, 0, 360, 255, -1)

    # C Shape
    main_canvas = cv2.rectangle(main_canvas, (200, 20), (230, 30), 255, -1)
    main_canvas = cv2.rectangle(main_canvas, (200, 60), (230, 70), 255, -1)
    main_canvas = cv2.rectangle(main_canvas, (200, 25), (210, 65), 255, -1)

    # Inclined rectangle

    pt_0 = (48, 300 - 108)
    pt_1 = point_2(pt_0, 150, 35)
    pt_2 = point_2(pt_1, 20, 90 + 35)
    pt_3 = point_2(pt_2, 150, 180 + 35)

    rectangle = np.array([pt_0, pt_1, pt_2, pt_3])

    cv2.fillConvexPoly(main_canvas, rectangle, 255)

    # 6 vetex Polygon
    pt_0 = (400 - 72, 300 - 63)
    pt_1 = point_2(pt_0, 75, 45)
    pt_2 = point_2(pt_1, 55, 90)
    pt_6 = point_2(pt_0, 60, 45 + 90)
    pt_5 = point_2(pt_6, 56, 45)
    pt_4 = point_2(pt_5, 27, 340)

    trapezium = np.array([pt_0, pt_1, pt_4, pt_5, pt_6])

    cv2.fillConvexPoly(main_canvas, trapezium, 255)

    triangle = np.array([pt_1, pt_2, pt_4])

    cv2.fillConvexPoly(main_canvas, triangle, 255)

    obstacle_list = []

    for i in range(main_canvas.shape[0]):
        for j in range(main_canvas.shape[1]):
            if main_canvas[i, j] == 255:
                obstacle_list.append(np.array((i, j)))

    return main_canvas, obstacle_list
