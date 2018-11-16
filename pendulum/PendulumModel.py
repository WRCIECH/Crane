#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, '../')
# try:
#     from Signals import step_function, impulse_function, ramp_function
# except:
#     raise


class PendulumModel():

    def __init__(self, list_of_charts):
        self.list_of_charts = list_of_charts

    def _set_up_system(self, l, g, m, kp, kd, theta, omega, method,
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

        self.td = -1.0
        self.tr_1090 = -1.0

        self.tr_0595 = -1.0
        self.tr_0100 = -1.0
        self.tp = -1.0
        self.over = -1.0
        self.ts_2 = -1.0
        self.ts_5 = -1.0

        self.l = l
        self.g = g
        self.m = m
        self.kp = kp
        self.kd = kd
        self.theta0 = theta
        self.theta = theta
        self.omega = omega

        self.yd = np.pi/4.0
        self.yd_prim = 0
        self.yd_bis = 0

        self.calculate_solution()

    def do_time_step(self):
        if (self.current_step > self.steps):
            self.current_step = 0
            self.t = 0
        self.theta = self.sol[self.current_step][0]
        self.omega = self.sol[self.current_step][1]
        self.current_step += 1
        self.t += self.dt

    def calculate_control(self, t1, t2, t):
        u = 0.0
        if self.control == "PID":
            pass
        elif self.control == "PolePlacement":
            u += self.g/self.l*np.sin(t1)
            u += -self.kp*(t1-self.yd) - self.kd*(t2-self.yd_prim)
            u *= self.l*self.m
        elif self.control == "Stabilizacja 0":
            u += self.g/self.l*np.sin(t1)
            u += -self.kp*t1 - self.kd*t2
        elif self.control == "Stabilizacja PI_5":
            angle = np.pi/5.0
            u += self.g/self.l*np.sin(t1)
            u += -self.kp*(t1+angle) - self.kd*t2

        return u

    def calculate_rise_time(self, start_v, end_v):

        tr_start = -1
        tr_end = -1
        i = 0

        if self.theta0 <= self.yd:
            for theta, omega in self.sol:
                if theta >= (self.theta0+end_v*(abs(self.yd-self.theta0))):
                    tr_end = self.przedzial[i]
                    return (tr_end - tr_start)
                elif theta >= self.theta0+start_v*(abs(self.theta0-self.yd)):
                    if tr_start < 0:
                        tr_start = self.przedzial[i]
                i += 1
        else:
            for theta, omega in self.sol:
                if theta <= self.yd+start_v*(abs(self.theta0-self.yd)):
                    tr_end = self.przedzial[i]
                    return (tr_end - tr_start)
                elif theta <= self.yd+end_v*(abs(self.theta0-self.yd)):
                    if tr_start < 0:
                        tr_start = self.przedzial[i]
                i += 1
        return -2.0

    def calculate_peak_time(self):

        if self.theta0 <= self.yd:
            i = np.argmax(self.sol[:, 0])
            self.tp = self.przedzial[i]
            self.over = (self.sol[i][0]-self.sol[-1][0])/self.sol[-1][0]*100
        else:
            i = np.argmin(self.sol[:, 0])
            self.tp = self.przedzial[i]
            self.over = (self.sol[i][0]-self.sol[-1][0])/self.sol[-1][0]*100

    def calculate_setting_time(self, val):
        ts = -1.0
        i = 0
        for theta, omega in self.sol:
            if theta >= (1.0-val)*self.yd and theta <= (1.0+val)*self.yd:
                if ts < 0.0:
                    ts = self.przedzial[i]
            else:
                ts = -1.0
            i += 1
        return ts

    def calculate_time_parameters(self):

        self.td = self.calculate_rise_time(0.0, 0.5)
        self.tr_1090 = self.calculate_rise_time(0.1, 0.9)
        self.tr_0595 = self.calculate_rise_time(0.05, 0.95)
        self.tr_0100 = self.calculate_rise_time(0.0, 1.0)
        self.calculate_peak_time()
        self.ts_2 = self.calculate_setting_time(0.02)
        self.ts_5 = self.calculate_setting_time(0.05)

    def pendulum_equations(self, y, t):
        t1, t2 = y
        u = self.calculate_control(t1, t2, t)

        dydt = [t2, -self.g/self.l*np.sin(t1)+u]

        return dydt

    def calculate_solution(self):
        x0 = [self.theta, self.omega]
        self.przedzial = np.linspace(0, self.steps*self.dt, self.steps+1)
        self.sol = odeint(self.pendulum_equations, x0, self.przedzial)

        self.calculate_time_parameters()
        self.draw_diagram_by_t()
        self.draw_diagram_phase()
        self.draw_diagram_y_x()
        self.list_of_charts[0].draw()
        self.list_of_charts[1].draw()
        self.list_of_charts[2].draw()

    def draw_diagram_by_t(self):
        plt.figure(1)
        plt.cla()
        plt.plot(self.przedzial, self.sol[:, 0], 'g', label='theta(t)')
        plt.plot(self.przedzial, self.sol[:, 1], 'black', label='omega(t)')
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum(t)')
            plt.savefig(name_of_file)

    def draw_diagram_phase(self):
        plt.figure(2)
        plt.cla()
        plt.plot(self.sol[:, 0], self.sol[:, 1], 'g', label='Omega(theta)')
        plt.legend(loc='best')
        plt.xlabel('x/theta')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum_phase')
            plt.savefig(name_of_file)

    def draw_diagram_y_x(self):
        plt.figure(3)
        plt.cla()
        thetas = self.sol[:, 0]
        ys = []
        for t in thetas:
            y = -self.l*np.cos(t)
            ys.append(y)
        plt.plot(self.sol[:, 0], ys, 'r', label='y(x)')
        plt.legend(loc='best')
        plt.xlabel('x')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum_y_x')
            plt.savefig(name_of_file)

    def generate_name_of_file(self, title):
        name_of_file = "./diagrams/"
        name_of_file += title + "_"
        name_of_file += "m_" + str(self.m) + "_"
        name_of_file += "l_" + str(self.l) + "_"
        name_of_file += "g_" + str(self.g) + "_"
        name_of_file += "theta0_" + str(self.theta) + "_"
        name_of_file += "omega0_" + str(self.omega) + "_"
        name_of_file += "dt_" + str(self.dt) + "_"
        name_of_file += "method_" + str(self.method)
        name_of_file += "control_" + str(self.control)
        name_of_file += "kp_" + str(self.kp)
        name_of_file += "kd_" + str(self.kd)
        name_of_file += ".jpg"
        return name_of_file
