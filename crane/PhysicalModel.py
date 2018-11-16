#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------
import numpy as np
from scipy.integrate import ode
import matplotlib.pyplot as plt
from ControlModels import ControlModel, ControlModelPD, ControlModelPD_2, \
    ControlModelKat, ControlModelSlidingMode, ControlModelUnderactuated
from NoiseModels import NoiseModel, NoiseModelContinuous


def select_control_model(control):

    if control == "PD":
        return ControlModelPD()
    elif control == "PD_2":
        return ControlModelPD_2()
    elif control == "KAT":
        return ControlModelKat()
    elif control == "SLIDING_MODE":
        return ControlModelSlidingMode()
    elif control == "UNDERACTUATED":
        return ControlModelUnderactuated()
    else:
        return ControlModel()


def select_noise_model(noise):
    if noise == 1:
        return NoiseModelContinuous()
    else:
        return NoiseModel()


class CranePhysicalModel():

    def __init__(self, list_of_charts):
        self.list_of_charts = list_of_charts

    def _set_up_system(self, l, g, m, M, kp, kd, A, f, theta, omega, x, V,
                       method, control, noise, dt, tmax, save_charts):

        self.save_charts = save_charts
        self.method = method
        self.control = control
        if noise=="Continuous":
            self.noise = 1
        else:
            self.noise = 0
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
        self.A = A
        self.f = f

        self.control_model = select_control_model(self.control)
        self.noise_model = select_noise_model(self.noise)

        self.control_model.set_values(self.l, self.g, self.m, self.M, self.kp,
                                      self.kd, self.A, self.f)
        x0 = [x, V, theta, omega]
        t0 = 0.0
        self.przedzial = np.linspace(0, self.steps*self.dt, self.steps+1)

        s = ode(self.crane_equations, jac=None)
        s.set_integrator(self.method, atol=0.1)
        s.set_initial_value(x0, t0)

        self.sol = np.ndarray(shape=(len(self.przedzial), 8), dtype=float)
        i = 0

        y = -self.l*np.cos(x0[2])
        yd = self.l*x0[3]*np.sin(x0[2])
        (u_1, u_2) = self.control_model.calculate_control(x0[0], x0[1],
                                                          x0[2], x0[3], s.t)
        self.sol[0] = [x0[0], x0[1], x0[2], x0[3], y, yd, u_1, u_2]
        while s.successful() and i < (len(self.przedzial)-1):
            s.integrate(s.t+self.dt)
            i = i+1
            y = -self.l*np.cos(s.y[2])
            yd = self.l*s.y[3]*np.sin(s.y[2])
            (u_1, u_2) = self.control_model.calculate_control(s.y[0], s.y[1],
                                                              s.y[2], s.y[3], s.t)

            self.sol[i] = [s.y[0], s.y[1], s.y[2], s.y[3], y, yd, u_1, u_2]

        self.draw_diagrams()

    def crane_equations(self, t, y):
        x1, x2, t1, t2 = y
        sin = np.sin(t1)
        cos = np.cos(t1)
        M = self.M
        m = self.m
        g = self.g
        l = self.l

        # sterowanie
        (u_1, u_2) = self.control_model.calculate_control(x1, x2, t1, t2, t)

        # zakłócenia
        (d_1, d_2) = self.noise_model.calculate_noise(x1, x2, t1, t2, t)

        mian = (M + m*sin*sin)

        f1 = x2
        f2 = (m*g*sin*cos+m * l*t2*t2*sin)/mian
        f3 = t2
        f4_numerator = (m*g*sin*cos*cos + m*l*t2*t2*sin*cos)
        f4 = -f4_numerator/(l*mian)-g/l*sin

        # wektorowo: x_prim = f + g*(u+delta)
        f = [f1, f2, f3, f4]
        G = [
            [0.0, 0.0],
            [1.0/mian, -cos/(mian)],
            [0.0, 0.0],
            [- cos/(l*mian), (1.0/(l*m)+cos*cos/(l*mian))]
        ]

        u = [u_1+d_1, u_2+d_2]
        dydt = f + np.matmul(G, u)

        return dydt

    def do_time_step(self):
        if (self.current_step > self.steps):
            self.current_step = 0
            self.t = 0
        self.x = self.sol[self.current_step][0]
        self.V = self.sol[self.current_step][1]
        self.theta = self.sol[self.current_step][2]
        self.omega = self.sol[self.current_step][3]
        self.current_step += 1
        self.t += self.dt

    def draw_diagrams(self):
        # self.draw_diagram_by_t()
        self.draw_diagram_moments()
        self.draw_diagram_phase()
        self.draw_diagram_y_x()
        self.list_of_charts[0].draw()
        self.list_of_charts[1].draw()
        self.list_of_charts[2].draw()

    def draw_diagram_by_t(self):
        plt.figure(1)
        plt.cla()
        plt.plot(self.przedzial, self.sol[:, 0], 'r', label='x(t)')
        plt.plot(self.przedzial, self.sol[:, 1], 'b', label='V(t)')
        plt.plot(self.przedzial, self.sol[:, 2], 'g', label='theta(t)')
        plt.plot(self.przedzial, self.sol[:, 3], 'black', label='omega(t)')
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('t')
            plt.savefig(name_of_file)

    def draw_diagram_phase(self):
        plt.figure(1)
        plt.cla()
        plt.plot(self.sol[:, 0], self.sol[:, 1], 'r', label='V(x)')
        plt.plot(self.sol[:, 2], self.sol[:, 3], 'g', label='Omega(theta)')
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('phase')
            plt.savefig(name_of_file)

    def draw_diagram_moments(self):
        plt.figure(2)
        plt.cla()
        plt.plot(self.przedzial, self.sol[:, 6], 'r', label='F(t)')
        plt.plot(self.przedzial, self.sol[:, 7], 'g', label='M(t)')
        plt.legend(loc='best')
        plt.xlabel('x/theta')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('moments')
            plt.savefig(name_of_file)

    def draw_diagram_y_x(self):
        plt.figure(3)
        plt.cla()
        xs = self.sol[:, 0]
        yds = []

        for x in xs:
            yd = -self.A*np.cos(self.f*x)
            yds.append(yd)

        plt.plot(self.sol[:, 0], self.sol[:, 4], 'r', label='y(x)')
        plt.plot(self.sol[:, 0], yds, 'g', label='yd(x)')

        plt.legend(loc='best')
        plt.xlabel('x')
        plt.grid()

        if self.save_charts is True:
            name_of_file = self.generate_name_of_file('yx')
            plt.savefig(name_of_file)

    def generate_name_of_file(self, title):
        name_of_file = "./diagrams/"
        name_of_file += title + "_"
        # name_of_file += "m_" + str(self.method)
        name_of_file += "c_" + str(self.control)
        name_of_file += "n_" + str(self.noise)
        name_of_file += ".jpg"
        return name_of_file
