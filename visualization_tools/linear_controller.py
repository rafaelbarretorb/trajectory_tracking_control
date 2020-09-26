
""" 
Python code based on the MATLAB script of Example 3.9, Listing 3.8, from the book "w_fheeled Mobile Robotics".
"""

import numpy as np

PI = np.pi

Ts = 0.05  # Sampling time
Td = 30  # Simulation duration time
t = np.arange(0, Td+Ts, Ts)
q = np.array([ [1.1], [0.8], [0]])

# Reference
w_f = 2*PI/30  # Angular frequency

xRef = 1.1 + 0.7*np.sin(w_f*t)
yRef = 0.9 + 0.7*np.sin(2*freq*t) 

dxRef = w_f*0.7*np.cos(w_f*t)
dyRef = 2*w_f*0.7*np.cos(2*w_f*t)

ddxRef = -(w_f**2)*2*0.7*np.sin(w_f*t)
ddyRef = -4*(w_f**2)*0.7*np.sin(2*w_f*t)

qRef = np.array([ [xRef], [yRef], [np.arctan2(dyRef, dxRef] ])  # Reference trajectory

vRef = np.array([np.sqrt(dxRef**2 + dyRef**2)])
wRef = np.array([ (dxRef*ddyRef - dyRef*ddxRef)/(dxRef**2 + dyRef**2) ])

uRef = np.array([ [vRef], [wRef] ])


for i in range(t.size):
    e = np.array([ [np.cos(q[3][0]), np.sin(q[3][0]), 0],
                   [-np.sin(q[3][0]), np.cos(q[3][0]), 0] 
                   [0, 0, 1] ])*(qRef - q)
    
    
