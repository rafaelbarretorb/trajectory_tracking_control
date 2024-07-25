# Python 3 code based on the MATLAB script of Example 3.9, Listing 3.8, from book "Wheeled Mobile
# Robotics".

import matplotlib.pyplot as plt
import numpy as np
import yaml

from helper_functions import wrap_to_pi
from random import random

plt.rcParams.update({'font.size': 15})


def rand():
    return (random() - 0.5)*2


def plot():
    pass


def compute_reference():
    pass


def main():
    with open('config.yaml', 'r') as file:
        linear_control_params = yaml.safe_load(file)

    params = linear_control_params['linear_controller']

    enable_noise = params['enable_noise']
    Ts = params['time']['sampling']  # Sampling time
    Td = params['time']['duration']  # Simulation duration
    t = np.arange(0, Td+Ts, Ts)
    q = np.array([
                  [params['start_pose']['x']],
                  [params['start_pose']['y']],
                  [params['start_pose']['phi']]
                ])

    # Control design parameters
    zeta = params['control_design']['zeta']
    g = params['control_design']['g']

    # Reference
    w_f = 2*(np.pi)/Td  # Angular frequency

    # TODO:
    xRef = 1.1 + 0.7*np.sin(w_f*t)
    yRef = 0.9 + 0.7*np.sin(2*w_f*t) 

    dxRef = w_f*0.7*np.cos(w_f*t)
    dyRef = 2*w_f*0.7*np.cos(2*w_f*t)

    ddxRef = -(w_f**2)*2*0.7*np.sin(w_f*t)
    ddyRef = -4*(w_f**2)*0.7*np.sin(2*w_f*t)

    qRef = np.stack((xRef, yRef, np.arctan2(dyRef, dxRef)), axis=0)  # Reference Pose

    vRef = np.sqrt(dxRef**2 + dyRef**2)
    wRef = (dxRef*ddyRef - dyRef*ddxRef)/(dxRef**2 + dyRef**2)
    uRef = np.stack((vRef, wRef), axis=0)

    q_log = list()
    v_log = list()
    w_log = list()
    Kx_log = list()
    Ky_log = list()

    noise_old = np.array([[0.0], [0.0], [0.0]])

    for i in range(t.size):
        TransMatrix = np.array([[np.cos(q[2][0]), np.sin(q[2][0]), 0],
                                [-np.sin(q[2][0]), np.cos(q[2][0]), 0],
                                [0, 0, 1]])

        e = TransMatrix.dot(qRef[:, [i]] - q)

        e[2][0] = wrap_to_pi(e[2][0])

        # Current reference inputs
        vRef_curr = uRef[:, [i]][0][0]
        wRef_curr = uRef[:, [i]][1][0]

        # CONTROL
        eX = e[0][0]
        eY = e[1][0]
        ePhi = e[2][0]

        # Gains
        if params['control_design']['const_gains'] is True:
            Kx = params['control_design']['k_x']
            Ky = params['control_design']['k_y']
            Kphi = params['control_design']['k_phi']
        else:
            Kx = 2*zeta*np.sqrt(wRef_curr**2 + g*(vRef_curr**2))
            Kphi = Kx
            Ky = g*vRef_curr

        # Feedforward and feedback
        v = vRef_curr*np.cos(ePhi) + Kx*eX
        w = wRef_curr + Ky*eY + Kphi*ePhi

        # Robot motion simulation
        dq = np.array([[v*np.cos(q[2][0])], [v*np.sin(q[2][0])], [w]])

        # Noise
        x_noise = 0.75
        y_noise = 0.75
        phi_noise = 0.75
        noise = noise_old + np.array([[x_noise],
                                      [y_noise],
                                      [1.0]]) * np.array([[rand()],
                                                          [rand()],
                                                          [1.0]])

        # phi noise is not accumulative and average value 0 (does not work accumulatives)
        noise[2][0] = phi_noise*rand()

        if enable_noise:
            q = q + Ts*dq*(np.ones([3, 1]) + noise)
        else:
            q = q + Ts*dq

        q[2][0] = wrap_to_pi(q[2][0])

        # Data Log
        q_log.append([q[0][0], [q[1][0]]])
        v_log.append(v)
        w_log.append(w)
        Kx_log.append(Kx)
        Ky_log.append(Ky)

    # PLOT
    # Figure 1
    plt.figure(1, figsize=(8, 8))

    ref,  = plt.plot(xRef, yRef, 'r--', label='reference trajectory')
    robot, = plt.plot(*zip(*q_log), label='robot position')
    plt.xlim([0.0, 2.0])
    plt.ylim([0.0, 2.0])
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend([ref, robot], ['reference trajectory', 'robot position'])

    # Figure 2
    plt.figure(2, figsize=(8, 8))

    plt.subplot(2, 1, 1)
    v, = plt.plot(t, v_log)
    ref_v, = plt.plot(t, vRef, 'r--')
    plt.xlabel('t (s)')
    plt.ylabel('v (m/s)')
    plt.legend([ref_v, v], ['reference linear velocity', 'robot linear velocity'])

    plt.subplot(2, 1, 2)
    w, = plt.plot(t, w_log)
    ref_w, = plt.plot(t, wRef, 'r--')
    plt.xlabel('t (s)')
    plt.ylabel('$\mathbf{\omega}$ (m/s)')
    plt.legend([ref_w, w], ['reference angular velocity', 'robot angular velocity'])

    plt.show()


if __name__ == "__main__":
    main()
