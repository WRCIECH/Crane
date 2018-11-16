#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
from DoublePendulumModel import DoublePendulumModel


class TestCreatingAppriopriateName(unittest.TestCase):

    def setUp(self):
        self.pendulum = DoublePendulumModel()
        self.pendulum._set_up_system(
            theta=0.5, omega=0.3, g=9.81, M=3.0, m=2.0, x=0.0, V=0.0,
            length=2.0, method='Euler', dt=0.01, draw_diagrams=False)

    def test_saving_to_file(self):
        name_of_file = "./diagrams/kokos_"
        name_of_file += "M_3.0_"
        name_of_file += "m_2.0_"
        name_of_file += "l_2.0_"
        name_of_file += "g_9.81_"
        name_of_file += "x0_0.0_"
        name_of_file += "theta0_0.5_"
        name_of_file += "V0_0.0_"
        name_of_file += "omega0_0.3_"
        name_of_file += "dt_0.01_"
        name_of_file += "method_Euler"
        name_of_file += ".jpg"
        self.assertEqual(name_of_file,
                         self.pendulum.generate_name_of_file('kokos'))


if __name__ == '__main__':
    unittest.main()
