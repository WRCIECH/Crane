#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2018, Wojciech Rembelski
# -----------------------------------------------------------------------------
import random


class NoiseModel:
    def __init__(self):
        self.prev_noiseF = 0

    def calculate_noise(self, x1, x2, t1, t2, t):
        return (0.0, 0.0)


class NoiseModelContinuous(NoiseModel):
    def calculate_noise(self, x1, x2, t1, t2, t):
        max_F_noise = 8.0
        dnoise = 1.0
        Fnoise = 0

        deviation = random.randint(-1, 1)
        Fnoise = self.prev_noiseF + deviation*dnoise
        if abs(Fnoise) > max_F_noise:
            Fnoise = self.prev_noiseF

        self.prev_noiseF = Fnoise
        return (Fnoise, 0.0)
