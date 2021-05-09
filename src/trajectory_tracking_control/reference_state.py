#!/usr/bin/env python

# Copyright 2020

import rospy
import numpy as np
import math
from scipy import interpolate
import copy


class ReferenceStates():
    """
    A class used to generate the reference states of the trajectory
    tracking control algorithm.

    ...

    Attributes
    ----------
    path : list of points
        a list of (x,y) n points [[x1,y1], [x2,y2], ... , [xn,yn]]
    vel_avg : float
        the average velocity of the trajectory
    t_s : float
        the sampling time

    Methods
    -------
    bspline(cv, n=100, degree=3, periodic=False)
        Generate the trajectory using a B-Spline interpolation.
    """
    def __init__(self, path, vel_avg, t_s):
        self.path = path
        self.vel_avg = vel_avg
        self.t_s = t_s

        self.rows_size = 6
        self.columns_size = 0
        self.traj_length = 0.0

    def compute_trajectory_length(self):
        """ ."""
        length = 0.0
        for i in range(len(self.path) - 1):
            dist = self.distance(self.path[i][0], self.path[i][1], self.path[i+1][0], self.path[i+1][1])
            length = length + dist

        return length

    def compute_reference_states(self):
        """ ."""

        data = np.array(self.path)

        # Trajectory Length
        self.traj_length = self.compute_trajectory_length()

        traj_duration = self.traj_length/self.vel_avg

        # distance step = Velocity_avg*Sampling time
        dist_step = self.vel_avg * self.t_s

        # Spline Size
        traj_size = int(self.traj_length/dist_step)

        rospy.loginfo("TEST 1")
        # Spline
        spline_curve, cv = self.make_spline_curve(0.5, 0.35, traj_size)

        rospy.loginfo("TEST 2")
        x, y = spline_curve.T

        self.columns_size = len(x)

        # Time Vector
        time = np.linspace(0, traj_duration, traj_size)

        dx = np.gradient(x)/np.gradient(time)
        dy = np.gradient(y)/np.gradient(time)
        ddx = np.gradient(dx)/np.gradient(time)
        ddy = np.gradient(dy)/np.gradient(time)

        states_ref = np.vstack((x, y, dx, dy, ddx, ddy))
        rospy.loginfo("TEST 3")

        return states_ref

    def b_spline(self, cv, n=100, degree=3, periodic=False):
        """ Calculate n samples on a bspline

            cv :      Array ov control vertices
            n  :      Number of samples to return
            degree:   Curve degree
            periodic: True - Curve is closed
                    False - Curve is open

            output = np.array([])
        """
        # If periodic, extend the point array by count+degree+1
        cv = np.asarray(cv)
        count = len(cv)

        if periodic:
            factor, fraction = divmod(count+degree+1, count)
            cv = np.concatenate((cv,) * factor + (cv[:fraction],))
            count = len(cv)
            degree = np.clip(degree, 1, degree)

        # If opened, prevent degree from exceeding count-1
        else:
            degree = np.clip(degree, 1, count-1)

        # Calculate knot vector
        kv = None
        if periodic:
            kv = np.arange(0-degree, count+degree+degree-1)
        else:
            kv = np.clip(np.arange(count+degree+1)-degree, 0, count-degree)

        # Calculate query range
        u = np.linspace(periodic, (count-degree), n)

        # Calculate result
        return np.array(interpolate.splev(u, (kv, cv.T, degree))).T

    def distance(self, x1, y1, x2, y2):
        """ Euclidean distance between two points (x1,y1) and (x2,y2)."""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_columns_size(self):
        """."""
        return self.columns_size

    def get_rows_size(self):
        """."""
        return self.rows_size

    def get_trajectory_length(self):
        """."""
        return self.traj_length

    def insert_control_points(self, cv, max_distance, tolerance):
        """ ."""
        cv_copy = copy.deepcopy(cv)
        count = 0
        for i in range(1, cv.shape[0]):
            x1 = cv[i-1][0]
            y1 = cv[i-1][1]

            x2 = cv[i][0]
            y2 = cv[i][1]

            dist = self.distance((x1, y1), (x2, y2))
            
            if dist > max_distance:
                n = int(dist/max_distance)
                if dist - n*max_distance < tolerance*max_distance:
                    n = n - 1
                for j in range(n):
                    count = count + 1
                    x, y = self.steer((x1, y1), (x2, y2), max_distance, j + 1)
                    row = np.array([x, y])
                    cv_copy = np.insert(cv_copy, i - 1 + count, row, axis=0)

        return cv_copy

    def steer(self, p1, p2, epsilon, n):
        """ ."""
        theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + n * epsilon * math.cos(theta), p1[1] + n * epsilon * math.sin(theta)

    def make_spline_curve(self, max_distance, tolerance, size, insert_cv=True):
        spline_curve = list()
        data = np.array(self.path)
        if insert_cv:
            data = self.insert_control_points(data, max_distance, tolerance)
        spline_curve = b_spline(data, n=size, degree=3)

        return spline_curve, data
