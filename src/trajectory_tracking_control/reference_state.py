
# Copyright 2020

import numpy as np
import math
from scipy import interpolate

class ReferenceState():

	def __init__(self, path, vel_avg, t_s):
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
		self.path = path
		self.vel_avg = vel_avg
 
    def trajectory_length(self):
        """ ."""

        length = 0.0
        for i in range(len(self.path) - 1):
            dist = distance(self.path[i][0], self.path[i][1], self.path[i+1][0], self.path[i+1][1])
            length = length + dist
        
        return length

    def compute_reference_states(self):
        """ ."""

        data = np.array(self.path)

        # Trajectory Length
        traj_length = self.trajectory_length()

        traj_duration = traj_length/self.avg_vel

		# distance step = Velocity_avg * Sampling_time	
		dist_step = self.vel_avg*self.t_s

        # Spline Size
        traj_size = int(traj_length/dist_step)

        # Spline
        p = self.bspline(data, n=traj_size, degree=3)
        x, y = p.T

        # Time Vector
        time = np.linspace(0, traj_duration, traj_size) 

        dx = np.gradient(x)/np.gradient(time)
        dy = np.gradient(y)/np.gradient(time)
        ddx = np.gradient(dx)/np.gradient(time)
        ddy = np.gradient(dy)/np.gradient(time)

        return states_ref = np.vstack((x, y, dx, dy, ddx, ddy))


    def bspline(self, cv, n=100, degree=3, periodic=False):
        """ Calculate n samples on a bspline

            cv :      Array ov control vertices
            n  :      Number of samples to return
            degree:   Curve degree
            periodic: True - Curve is closed
                    False - Curve is open
        """
        # If periodic, extend the point array by count+degree+1
        cv = np.asarray(cv)
        count = len(cv)

        if periodic:
            factor, fraction = divmod(count+degree+1, count)
            cv = np.concatenate((cv,) * factor + (cv[:fraction],))
            count = len(cv)
            degree = np.clip(degree,1,degree)

        # If opened, prevent degree from exceeding count-1
        else:
            degree = np.clip(degree,1,count-1)

        # Calculate knot vector
        kv = None
        if periodic:
            kv = np.arange(0-degree,count+degree+degree-1)
        else:
            kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

        # Calculate query range
        u = np.linspace(periodic,(count-degree),n)

        # Calculate result
        return np.array(interpolate.splev(u, (kv,cv.T,degree))).T

	def distance(self, x1, y1, x2, y2):
		""" Euclidean distance between two points (x1,y1) and (x2,y2)."""
		return math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))