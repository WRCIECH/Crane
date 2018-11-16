#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2018, Wojciech Rembelski
# -----------------------------------------------------------------------------
import numpy as np
import math
from numpy.linalg import inv


class ControlModel():
    def set_values(self, l, g, m, M, kp, kd, A, f):
        self.l = l
        self.g = g
        self.m = m
        self.M = M
        self.kp = kp
        self.kd = kd
        self.A = A
        self.f = f

    def calculate_control(self, x1, x2, t1, t2, t):
        return (0.0, 0.0)


class ControlModelPD(ControlModel):

    def calculate_control(self, x1, x2, t1, t2, t):
        l = self.l
        g = self.g
        m = self.m
        M = self.M
        kp = self.kp
        kd = self.kd
        A = self.A
        f = self.f
        u_1 = 0.0
        u_2 = 0.0

        kd_part = kd*(l*t2*np.sin(t1)-A*x2*f*np.sin(f*x1))
        kp_part = kp*(-l*np.cos(t1)+A*np.cos(f*x1))
        rest = l*t2*t2*np.cos(t1) - A*x2*x2*f*f*np.cos(f*x1)
        nu_1 = 0
        nu_2 = 0
        Z = - rest - kd_part - kp_part
        if abs(t1) < 0.00001:
            nu_1 = 0
            nu_2 = 0
        else:
            c = -0.5
            nu_1 = -kd*(x2-c) - kp*(x1-c*t)
            nu_2 = (Z+A*nu_1*f*np.sin(f*x1))/(l*np.sin(t1))
        u_1 = -m*l*t2*t2*np.sin(t1) + (m+M)*nu_1 + m*l*np.cos(t1)*nu_2
        u_2 = m*g*np.sin(t1) + m*np.cos(t1)*nu_1 + m*l*nu_2

        return (u_1, u_2)


class ControlModelPD_2(ControlModel):

    def calculate_control(self, x1, x2, t1, t2, t):
        l = self.l
        g = self.g
        m = self.m
        M = self.M
        kp = self.kp
        kd = self.kd
        A = self.A
        f = self.f
        u_1 = 0.0
        u_2 = 0.0

        c = 0.5
        nu_1 = -kd*(x2+c) - kp*(x1 + c*t)

        cmian = 1.0-A*A/(l*l)*np.cos(f*c*t)*np.cos(f*c*t)
        ts = math.acos(A/l*np.cos(f*c*t))
        ts_prim = (A/l*f*c*np.sin(f*c*t))/(np.sqrt(cmian))
        c1 = f*c*c*np.cos(f*c*t)*np.sqrt(cmian)
        c2 = 1.0/cmian*A*A*c*c/(l*l)*np.sin(f*c*t)*np.sin(f*c*t)*np.cos(f*c*t)
        ts_bis = A/l*f*(c1+c2)/(cmian)

        nu_2 = ts_bis - kd*(t2-ts_prim) - kp*(t1-ts)
        u_1 = -m*l*t2*t2*np.sin(t1) + (m+M)*nu_1 + m*l*np.cos(t1)*nu_2
        u_2 = m*g*np.sin(t1) + m*np.cos(t1)*nu_1 + m*l*nu_2

        return (u_1, u_2)


class ControlModelKat(ControlModel):

    def calculate_control(self, x1, x2, t1, t2, t):
        l = self.l
        g = self.g
        m = self.m
        M = self.M
        kp = self.kp
        kd = self.kd
        f = self.f
        u_1 = 0.0
        u_2 = 0.0

        c = 0.1
        nu_1 = -kd*(x2-c) - kp*(x1-c*t)
        nu_2 = -f*f*np.sin(f*t) - kp*(t2-f*np.cos(f*t)) - kp*(t1-np.sin(f*t))
        u_1 = -m*l*t2*t2*np.sin(t1) + (m+M)*nu_1 + m*l*np.cos(t1)*nu_2
        u_2 = m*g*np.sin(t1) + m*np.cos(t1)*nu_1 + m*l*nu_2

        return (u_1, u_2)


class ControlModelSlidingMode(ControlModel):

    def calculate_control(self, x1, x2, t1, t2, t):
        l = self.l
        g = self.g
        m = self.m
        M = self.M
        A = self.A
        f = self.f
        kp = self.kp
        kd = self.kd

        lambda_1 = 1.0
        lambda_2 = 1.0
        k1 = kp
        k2 = kd

        c = 0.5
        xs = -c*t
        xs_prim = -c
        xs_bis = 0.0

        cmian = 1.0-A*A/(l*l)*np.cos(f*c*t)*np.cos(f*c*t)
        ts = math.acos(A/l*np.cos(f*c*t))
        c1 = f*c*c*np.cos(f*c*t)*np.sqrt(cmian)
        c2 = 1.0/cmian*A*A*c*c/(l*l)*np.sin(f*c*t)*np.sin(f*c*t)*np.cos(f*c*t)
        ts_prim = (A/l*f*c*np.sin(f*c*t))/(np.sqrt(cmian))
        ts_bis = A/l*f*(c1+c2)/(cmian)

        s1 = x2 - xs_prim + lambda_1*(x1-xs)
        s2 = t2 - ts_prim + lambda_2*(t1-ts)

        sin = np.sin(t1)
        cos = np.cos(t1)
        mian = (M + m*sin*sin)

        G = [
            [1.0/mian, -cos/(mian)],
            [- cos/(l*mian), (1.0/(l*m)+cos*cos/(l*mian))]]
        G_inv = inv(G)

        f_p = [t2*t2*np.sin(t1), t2*t2*np.sin(t1)*np.cos(t1)]
        F = [m*g/M+t2*t2*np.sin(t1), t2*t2*np.sin(t1)*np.cos(t1)*(1+m/M) + g*(m+M)/(l*M)]

        rest = [xs_bis - lambda_1*(x2-xs_prim) - f_p[0] - (k1+F[0])*np.sign(s1),
                ts_bis - lambda_2*(t2-ts_prim) - f_p[1] - (k2+F[1])*np.sign(s2)]

        u = np.matmul(G_inv, rest)

        return (u[0], u[1])


class ControlModelUnderactuated(ControlModel):

    def calculate_control(self, x1, x2, t1, t2, t):
        l = self.l
        g = self.g
        m = self.m
        M = self.M
        kp = self.kp
        kd = self.kd
        A = self.A
        f = self.f
        u_1 = 0.0
        u_2 = 0.0

        if abs(np.cos(t1)) < 0.01 or abs(x1) < 0.001:
            nu_1 = -kd*(x2+0.5) - kp*(x1+0.5*t)
            # nu_1 = 0  # -kp*(x2)
        else:
            sin = np.sin(f*x1)
            cos = np.cos(f*x1)
            umian = np.sqrt(1-A*A/(l*l)*cos*cos)
            t_s = math.acos(A/l*cos)
            t_s_prim = (A*f*x2*sin)/(l*umian)
            p1 = (-g*np.tan(t1)*sin+x2*x2*f*cos)*umian
            p2 = -1.0/umian*A*A/(l*l)*cos*sin*sin*f*x2*x2
            t_s_bis_part = A/l*f*(p1+p2)/(umian*umian)
            nu_2_factor = (1.0+(A*f*sin)/(np.cos(t1)*umian))

            kd_part = kd*(t2-t_s_prim)
            kp_part = kp*(t1-t_s)

            k3 = 50.0
            nu_2 = (-kd_part-kp_part+t_s_bis_part)/nu_2_factor + k3*np.cos(t1)*x2
            nu_1 = -g*np.tan(t1) - l/np.cos(t1)*nu_2

        with open('out.txt', 'a') as f:
            print >> f, t2, x2
        u_1 = -m*g*np.sin(t1)*np.cos(t1) - m*l*t2*t2*np.sin(t1) + (M+m*np.sin(t1)*np.sin(t1))*nu_1
        u_2 = 0

        return (u_1, u_2)


class ControlModelInputOutput(ControlModel):

    def calculate_control(self, x1, x2, y1, y2, t):
        u_1 = 0.0
        u_2 = 0.0

        return (u_1, u_2)
