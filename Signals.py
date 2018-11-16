#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------

"""
**********
Pendulum
Simulation of pendulum
**********
"""

from __future__ import division, print_function, absolute_import
# import numpy as np


def step_function(t):
    return 1.0


def impulse_function(t):
    return float("inf")


def ramp_function(t):
    return t
