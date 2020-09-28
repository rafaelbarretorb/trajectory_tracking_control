
""" 
Python code based on the MATLAB script of Example 3.9, Listing 3.8, from the book "w_fheeled Mobile Robotics".
"""

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 15})
from random import random

PI = np.pi

def rand():
    return (random() - 0.5)*2

def wrapToPi2(theta):
    """ """
    return (theta + np.pi) % (2 * np.pi) - np.pi

def wrapToPi(theta):
    """ """
    return np.arctan2(np.sin(theta), np.cos(theta))

def main():
    acc_lin_max = 0.25  # 0.25
    acc_ang_max = PI/6  # PI/12
    v_max = 0.50
    w_max = PI/6 # PI/6

    enable_noise = False
    Ts = 0.05  # Sampling time
    Td = 90  # Simulation duration time
    t = np.arange(0, Td+Ts, Ts)
    q = np.array([ [1.1], [0.8], [0]])

    # Reference
    w_f = 2*PI/Td  # Angular frequency

    xRef = 1.1 + 2.5*np.sin(w_f*t)
    yRef = 0.9 + 2.5*np.sin(2*w_f*t) 

    dxRef = w_f*2.5*np.cos(w_f*t)
    dyRef = 2*w_f*2.5*np.cos(2*w_f*t)

    ddxRef = -(w_f**2)*2*2.5*np.sin(w_f*t)
    ddyRef = -4*(w_f**2)*2.5*np.sin(2*w_f*t)

    qRef = np.stack((xRef, yRef, np.arctan2(dyRef, dxRef)), axis=0)  # Reference trajectory

    vRef = np.sqrt(dxRef**2 + dyRef**2)
    wRef = (dxRef*ddyRef - dyRef*ddxRef)/(dxRef**2 + dyRef**2)
    uRef = np.stack((vRef, wRef), axis=0)

    # Print shape od the arrays
    # print "xRef : " + str(xRef.shape)
    # print "dxRef : " + str(dxRef.shape)
    # print "ddxRef : " + str(ddxRef.shape)
    # print "vRef : " + str(vRef.shape)
    # print "wRef : " + str(wRef.shape)
    # print "uRef : " + str(uRef.shape)

    q_log = list()
    v_log = list()
    w_log = list()
    Kx_log = list()
    Ky_log = list()
    x = 50
    noise_old = np.array([ [0.0], [0.0], [0.0]])
    v_old = 0.0
    w_old = 0.0
    for i in range(t.size):
        TransMatrix = np.array([ [np.cos(q[2][0])  , np.sin(q[2][0]) , 0],
                                 [-np.sin(q[2][0]) , np.cos(q[2][0]) , 0],
                                 [0                , 0               , 1] ])
        
        e = TransMatrix.dot(qRef[:, [i] ] - q)

        e[2][0] = wrapToPi(e[2][0])

        # Current reference inputs
        vRef_curr = uRef[:, [i]][0][0]
        wRef_curr = uRef[:, [i]][1][0]

        ### CONTROL ###
        eX = e[0][0]
        eY = e[1][0]
        ePhi = e[2][0]

        # Control design parameters
        zeta = 0.9
        g = 85

        # Gains
        Kx = 2*zeta*np.sqrt(wRef_curr**2 + g*(vRef_curr**2))
        Kphi = Kx
        Ky = g*vRef_curr

        # Feedforward and feedback
        v = vRef_curr*np.cos(ePhi) + Kx*eX
        acc_lin = (v - v_old)/Ts

        if abs(acc_lin) > acc_lin_max:
            acc_lin = np.sign(acc_lin)*acc_lin_max
        
        v = v_old + acc_lin*Ts
        if v > v_max:
            v = v_max

        w = wRef_curr + Ky*eY + Kphi*ePhi
        # w_tmp = w
        # acc_ang = (w - w_old)/Ts
        # if abs(acc_ang) > acc_ang_max:
        #     acc_ang = np.sign(acc_ang)*acc_ang_max
        
        # w = w_old + acc_ang*Ts
        # if abs(w) > w_max:
        #     w = np.sign(w)*w_max
        
        # print "w = " + str(w_tmp) + " w = " + str(w)

        # Robot motion simulation
        dq = np.array([ [v*np.cos(q[2][0])], [v*np.sin(q[2][0])], [w] ])

        # Noise
        x_noise = 0.75
        y_noise = 0.75
        phi_noise = 0.35
        noise = noise_old + np.array([ [x_noise], [y_noise], [1.0]]) * np.array([ [rand()], [rand()], [1.0]])
        noise[2][0] = phi_noise*rand() # phi noise is not accumulative and average value 0

        if enable_noise:
            q = q + Ts*dq*(np.ones([3,1]) + noise)
        else:
            q = q + Ts*dq

        q[2][0] = wrapToPi(q[2][0])
        # noise_old = noise

        # Data Log
        q_log.append([q[0][0], [q[1][0]]])
        v_log.append(v)
        w_log.append(w)
        Kx_log.append(Kx)
        Ky_log.append(Ky)

        v_old = v
        w_old = w

        # print "Noise:  x = " + str(noise[0][0]) + "  y = " + str(noise[1][0]) + "  phi= " + str(np.degrees(noise[2][0])) + " "

        # if i == x:
        #     print "e matrix shape = " + str(e.shape)
        #     print "eX = " + str(eX)
        #     print "eY = " + str(eY)
        #     print "ePhi = " + str(np.degrees(ePhi)) + " degrees"
        #     print "Kx = " + str(Kx)
        #     print "Ky = " + str(Ky)
        #     print "Kphi = " + str(Kphi)
        #     print "v = " + str(v)
        #     print "w = " + str(w)
        #     print "dq = " + str(dq)
        #     print "q = " + str(q)
        #     print "noise shape = " + str(noise.shape)

    #####  PLOT #####
    # Figure 1
    plt.figure(1, figsize=(8, 8))

    plt.plot(xRef, yRef, 'r--')
    plt.plot( *zip(*q_log) )
    plt.xlim([-4.0, 4.0])
    plt.ylim([-4.0, 4.0])
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')

    # Figure 2
    fig2, (ax1, ax2) = plt.subplots(2, figsize=(16, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, v_log)
    plt.plot(t, vRef, 'r--')
    plt.xlabel('t (s)')
    plt.ylabel('v (m/s)')

    plt.subplot(2, 1, 2)
    plt.plot(t, w_log)
    plt.plot(t, wRef, 'r--')
    plt.xlabel('t (s)')
    plt.ylabel('$\mathbf{\omega}$ (m/s)')

    # Figure 2
    fig3, (ax1, ax2) = plt.subplots(2, figsize=(16, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, Kx_log)
    plt.xlabel('t (s)')
    plt.ylabel('Kx')

    plt.subplot(2, 1, 2)
    plt.plot(t, Ky_log)
    plt.xlabel('t (s)')
    plt.ylabel('Ky')
    # plt.figure(3, figsize=(10,10))

    # plt.plot(xRef, yRef, 'r--')
    # plt.plot( *zip(*q_log) )
    # plt.xlim([0.0, 2.0])
    # plt.ylim([0.0, 2.0])
    # plt.xlabel('x (m)')
    # plt.ylabel('y (m)')

    plt.show()

if __name__ == "__main__":
    main()
