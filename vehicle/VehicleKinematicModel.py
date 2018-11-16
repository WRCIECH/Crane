#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt


class VehicleKinematicModel():

    def __init__(self, list_of_charts):
        self.list_of_charts = list_of_charts

    def _set_up_system(self, l, kp, kd, theta, fi, x, y, method,
                       control, dt, tmax, save_charts):

        self.save_charts = save_charts
        self.method = method
        self.control = control
        self.t = 0
        self.tmax = tmax
        self.dt = dt
        self.steps = int(self.tmax/self.dt)
        print("Liczba kroków: " + str(self.steps))
        self.current_step = 0

        self.x = x
        self.y = y
        self.l = l
        self.kp = kp
        self.kd = kd
        self.fi = fi
        self.theta = theta
        self.calculate_solution()

    def do_time_step(self):
        if (self.current_step > self.steps):
            self.current_step = 0
            self.t = 0
        self.x = self.sol[self.current_step][0]
        self.y = self.sol[self.current_step][1]
        self.theta = self.sol[self.current_step][2]
        self.fi = self.sol[self.current_step][3]
        self.current_step += 1
        self.t += self.dt

    def calculate_control(self, x, y, theta, fi, t):
        u1 = 1.0
        u2 = 0.0  # 1.0*np.cos(t)
        u = [u1, u2]

        return u

    def vehicle_equations(self, vec, t):
        x, y, theta, fi = vec
        u = self.calculate_control(x, y, theta, fi, t)
        xp = u[0]*np.cos(theta)*np.cos(fi)
        yp = u[0]*np.sin(theta)*np.cos(fi)
        thp = 1.0/self.l*np.sin(fi)
        fip = u[1]
        dydt = [xp, yp, thp, fip]

        return dydt

    def calculate_solution(self):
        x0 = [self.x, self.y, self.theta, self.fi]
        self.przedzial = np.linspace(0, self.steps*self.dt, self.steps+1)
        self.sol = odeint(self.vehicle_equations, x0, self.przedzial)

        self.draw_diagram_by_t()
        self.draw_diagram_phase()
        self.list_of_charts[0].draw()
        self.list_of_charts[1].draw()

    def draw_diagram_by_t(self):
        plt.figure(1)
        plt.cla()
        plt.plot(self.przedzial, self.sol[:, 0], 'r', label='x(t)')
        plt.plot(self.przedzial, self.sol[:, 1], 'g', label='y(t)')
        plt.plot(self.przedzial, self.sol[:, 2], 'b', label='theta(t)')
        plt.plot(self.przedzial, self.sol[:, 3], 'black', label='fi(t)')
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('vehicle_kinematic(t)')
            plt.savefig(name_of_file)

    def draw_diagram_phase(self):
        plt.figure(2)
        plt.cla()
        plt.plot(self.sol[:, 0], self.sol[:, 1], 'r', label='y(x)')
        plt.plot(self.sol[:, 1], self.sol[:, 3], 'g', label='fi(theta)')
        plt.legend(loc='best')
        plt.xlabel('x/theta')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('vehicle_kinematic_phase')
            plt.savefig(name_of_file)

    def generate_name_of_file(self, title):
        name_of_file = "./diagrams/"
        name_of_file += title + "_"
        name_of_file += "l_" + str(self.l) + "_"
        name_of_file += "x_" + str(self.x) + "_"
        name_of_file += "y_" + str(self.y) + "_"
        name_of_file += "theta_" + str(self.theta) + "_"
        name_of_file += "fi_" + str(self.fi) + "_"
        name_of_file += "dt_" + str(self.dt) + "_"
        name_of_file += "method_" + str(self.method)
        name_of_file += "control_" + str(self.control)
        name_of_file += "kp_" + str(self.kp)
        name_of_file += "kd_" + str(self.kd)
        name_of_file += ".jpg"
        return name_of_file
