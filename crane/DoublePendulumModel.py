#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt


class DoublePendulumModel():

    def __init__(self, list_of_charts):
        self.list_of_charts = list_of_charts

    def _set_up_system(self, l, g, m, M, kp, kd, theta, omega, x, V, method,
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
        self.V = V
        self.l = l
        self.g = g
        self.m = m
        self.M = M
        self.kp = kp
        self.kd = kd
        self.theta = theta
        self.omega = omega
        self.prev_u = 0.0
        self.ampl = self.l/4.0
        self.czest = 0.003

        self.calculate_solution()

    def do_time_step(self):
        if (self.current_step > self.steps):
            self.current_step = 0
            self.t = 0
        self.x = self.sol[self.current_step][0]
        self.theta = self.sol[self.current_step][1]
        self.V = self.sol[self.current_step][2]
        self.omega = self.sol[self.current_step][3]
        self.current_step += 1
        self.t += self.dt

    def calculate_control(self, x1, t1, x2, t2, t):
        u = 0.0
        y = -self.l*np.cos(t1)
        y_prim = self.l*t2*np.sin(t1)
        yd = -self.ampl*np.sin(self.czest*x1)
        yd_prim = -self.ampl*self.czest*x2*np.cos(x1)

        e = y - yd
        e_prim = y_prim - yd_prim
        if self.control == "PID":
            u = -self.kp*e - self.kd*e_prim

        elif self.control == "FL_PP":
            if abs(t1) > 0.001:
                ni = -self.kp*e - self.kd*e_prim
                mian = self.M + self.m*(1 - y**2/self.l**2)
                sq = np.sqrt(self.l**2-y**2)
                czl_1 = -ni*(mian)/(y*sq/self.l**2)
                czl_2 = -self.m/self.l*y_prim**2
                czl_3 = -self.m*self.g*sq*y/self.l**2
                czl_4 = self.g*sq/self.l*(mian)/y
                u = czl_1 + czl_2 + czl_3 + czl_4
                self.prev_u = u

            else:
                u = self.prev_u

            pass

        return u

    def pendulum_equations(self, y, t):
        x1, t1, x2, t2 = y
        u = self.calculate_control(x1, t1, x2, t2, t)
        lic_x2 = (u + self.m*self.g*np.sin(t1)*np.cos(t1)+self.m *
                  self.l*t2*t2*np.sin(t1))
        mian = (self.M+self.m*np.sin(t1)*np.sin(t1))

        ulam = (u + self.m*self.g*np.sin(t1)*np.cos(t1)+self.m*self.l*t2 *
                t2*np.sin(t1))
        liczt2 = ((-1.0)/(self.l))*np.cos(t1) * ulam

        # print("theta: " + str(t1/np.pi*180.0))
        dydt = [x2, t2, lic_x2 / mian, liczt2 / mian - self.g/self.l*np.sin(t1)]

        return dydt

    def calculate_solution(self):
        x0 = [self.x, self.theta, self.V, self.omega]
        self.przedzial = np.linspace(0, self.steps*self.dt, self.steps+1)
        self.sol = odeint(self.pendulum_equations, x0, self.przedzial)

        self.draw_diagram_by_t()
        self.draw_diagram_phase()
        self.draw_diagram_y_x()
        self.list_of_charts[0].draw()
        self.list_of_charts[1].draw()
        self.list_of_charts[2].draw()

    def draw_diagram_by_t(self):
        plt.figure(1)
        plt.cla()
        plt.plot(self.przedzial, self.sol[:, 0], 'r', label='x(t)')
        plt.plot(self.przedzial, self.sol[:, 1], 'g', label='theta(t)')
        plt.plot(self.przedzial, self.sol[:, 2], 'b', label='V(t)')
        plt.plot(self.przedzial, self.sol[:, 3], 'black', label='omega(t)')
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum(t)')
            plt.savefig(name_of_file)

    def draw_diagram_phase(self):
        plt.figure(2)
        plt.cla()
        plt.plot(self.sol[:, 0], self.sol[:, 2], 'r', label='V(x)')
        plt.plot(self.sol[:, 1], self.sol[:, 3], 'g', label='Omega(theta)')
        plt.legend(loc='best')
        plt.xlabel('x/theta')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum_phase')
            plt.savefig(name_of_file)

    def draw_diagram_y_x(self):
        plt.figure(3)
        plt.cla()
        thetas = self.sol[:, 1]
        xs = self.sol[:, 0]
        ys = []
        yds = []
        for t in thetas:
            y = -self.l*np.cos(t)
            ys.append(y)

        for x in xs:
            yd = -self.ampl*np.sin(self.czest*x)
            yds.append(yd)

        plt.plot(self.sol[:, 0], ys, 'r', label='y(x)')
        plt.plot(self.sol[:, 0], yds, 'g', label='yd(x)')

        plt.legend(loc='best')
        plt.xlabel('x')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('double_pendulum_y_x')
            plt.savefig(name_of_file)

    def generate_name_of_file(self, title):
        name_of_file = "./diagrams/"
        name_of_file += title + "_"
        name_of_file += "M_" + str(self.M) + "_"
        name_of_file += "m_" + str(self.m) + "_"
        name_of_file += "l_" + str(self.l) + "_"
        name_of_file += "g_" + str(self.g) + "_"
        name_of_file += "x0_" + str(self.x) + "_"
        name_of_file += "theta0_" + str(self.theta) + "_"
        name_of_file += "V0_" + str(self.V) + "_"
        name_of_file += "omega0_" + str(self.omega) + "_"
        name_of_file += "dt_" + str(self.dt) + "_"
        name_of_file += "method_" + str(self.method)
        name_of_file += "control_" + str(self.control)
        name_of_file += "kp_" + str(self.kp)
        name_of_file += "kd_" + str(self.kd)
        name_of_file += ".jpg"
        return name_of_file
