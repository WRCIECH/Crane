#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2017, Wojciech Rembelski
# -----------------------------------------------------------------------------
from __future__ import division, print_function, absolute_import
from vispy import visuals
from vispy.visuals import transforms
import logging
import string
import numpy as np
import traceback
from PyQt4 import QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from PendulumModel import PendulumModel

import sys
sys.path.insert(0, '../')
try:
    from VisualModel import VisualModel, MainWindowBase, SetupWidgetData
except:
    raise


logger = logging.getLogger(__name__)


class VPendulumModel(VisualModel):

    def __init__(self, list_of_charts, theta, omega, g, m, kp, kd,
                 l, scale, method, control, save_charts,
                 dt, tmax, font_size):

        VisualModel.__init__(self)

        self.model = PendulumModel(list_of_charts)
        self.list_of_charts = list_of_charts

        self.vis_length = l

        self._set_up_system(
            l=l, g=g, m=m, theta=theta, omega=omega, kp=kp,
            kd=kd, scale=scale, method=method, control=control,
            save_charts=save_charts, dt=dt, tmax=tmax, font_size=font_size
        )

        # Put up a text visual to display time info
        self.font_size = 24. if font_size is None else font_size
        self.text = visuals.TextVisual('0:00.00',
                                       color='white',
                                       pos=[15, 50, 0],
                                       anchor_x='left',
                                       anchor_y='bottom')
        self.text.font_size = self.font_size

        self.params = []
        self.params.append(visuals.TextVisual('td', color='white',
                                              pos=[15, 80, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('tr_1090', color='white',
                                              pos=[15, 100, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('tr_0595', color='white',
                                              pos=[15, 120, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('tr_0100', color='white',
                                              pos=[15, 140, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('tp', color='white',
                                              pos=[15, 160, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('over', color='white',
                                              pos=[15, 180, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('ts_2', color='white',
                                              pos=[15, 200, 0], anchor_x='left',
                                              anchor_y='bottom'))
        self.params.append(visuals.TextVisual('ts_5', color='white',
                                              pos=[15, 220, 0], anchor_x='left',
                                              anchor_y='bottom'))
        for param in self.params:
            param.font_size = 12

        self.hook = visuals.BoxVisual(width=0.25, height=0.25, depth=0.25,
                                      color='grey')
        self.hook.transform = transforms.MatrixTransform()
        self.hook.transform.scale((self.scale*self.model.l,
                                  self.scale*self.model.l, 0.0001))
        self.hook.transform.translate(self.center)

        self.rod = visuals.BoxVisual(width=self.vis_length/40,
                                     height=self.vis_length/40,
                                     depth=self.vis_length,
                                     color='green')
        self.rod.transform = transforms.MatrixTransform()
        self.rod.transform.translate([0.0, self.vis_length/2.0])
        self.rod.transform.scale((self.scale*self.model.l,
                                  self.scale*self.model.l, 0.0001))
        self.rod.transform.rotate(np.rad2deg(self.model.theta), (0, 0, 1))
        self.rod.transform.translate(self.center)

        # Append all the visuals
        self.visuals.append(self.rod)
        self.visuals.append(self.hook)
        self.visuals.append(self.text)
        for param in self.params:
            self.visuals.append(param)

    def on_timer(self, ev):
        millis_passed = int(100 * (self.model.t % 1))
        sec_passed = int(self.model.t % 60)
        min_passed = int(self.model.t // 60)

        new_theta = self.model.theta

        self.hook.transform.reset()
        self.hook.transform.scale((self.scale*self.model.l,
                                  self.scale*self.model.l, 0.0001))
        self.hook.transform.translate(self.center)
        self.hook.transform.translate([self.scale*self.model.l, 0.0])

        # Apply the necessary transformations to the rod
        self.rod.transform.reset()
        self.rod.transform.translate([0.0, self.vis_length/2.0])
        self.rod.transform.scale((self.scale*self.model.l,
                                  self.scale*self.model.l, 0.0001))
        self.rod.transform.rotate(np.rad2deg(new_theta), (0, 0, 1))
        self.rod.transform.translate(self.center)
        self.rod.transform.translate([self.scale*self.model.l, 0.0])

        # Update the timer with how long it's been
        self.text.text = '{:0>2d}:{:0>2d}.{:0>2d}'.format(min_passed,
                                                          sec_passed,
                                                          millis_passed)

        self.params[0].text = "td: " + str(self.model.td)
        self.params[1].text = "tr_1090: " + str(self.model.tr_1090)
        self.params[2].text = "tr_0595: " + str(self.model.tr_0595)
        self.params[3].text = "tr_0100: " + str(self.model.tr_0100)
        self.params[4].text = "tp: " + str(self.model.tp)
        self.params[5].text = "overshoot: " + str(self.model.over)
        self.params[6].text = "ts_2: " + str(self.model.ts_2)
        self.params[7].text = "ts_5: " + str(self.model.ts_5)

        self.update()
        self.model.do_time_step()

    def reset_parms(self, l, g, m, kp, kd, theta, omega, scale, method,
                    control, save_charts, dt, tmax, font_size):

        self._set_up_system(
            l=l, g=g, m=m, theta=theta, omega=omega, kp=kp,
            kd=kd, scale=scale, method=method, control=control,
            save_charts=save_charts, dt=dt, tmax=tmax, font_size=font_size
        )

    def _set_up_system(self, l, g, m, kp, kd, theta,
                       omega, scale, method, control, save_charts,
                       dt, tmax, font_size):

        self.model._set_up_system(l=l, g=g, m=m, kp=kp, kd=kd, theta=theta,
                                  omega=omega, method=method, control=control,
                                  save_charts=save_charts, dt=dt, tmax=tmax)

        for chart in self.list_of_charts:
            chart.draw()

        self.method = string.capwords(method, '-')

        self.font_size = font_size
        try:
            self.text.font_size = self.font_size
        except AttributeError:
            pass

        # Initialize constants for display
        self.scale = 50 if scale is None else scale


class MainWindow(MainWindowBase):

    def __init__(self, param=None):

        icon_name = 'img/icon_pendulum.png'
        window_name = 'Pendulum simulation'

        labels = [
            (0, 'system_parameters'),
            (1, 'initial_conditions'),
            (2, 'pid_parameters'),
            (3, 'display_parameters')]

        parameters = [('l',         0.1,      10.0,    'double', 5.0, 0.1, 0),
                      ('g',         1.0,      20.0,    'double', 9.81, 0.1, 0),
                      ('dt', 0.001,    1.0,     'double', 0.017, 0.001, 0),
                      ('tmax', 1.0,    100.0,     'double', 50.0, 1.0, 0),
                      ('theta', -np.pi, np.pi, 'double', np.pi/6.0, 0.01, 1),
                      ('omega', -50.0, 50.0, 'double', 0.0, 0.01, 1),
                      ('m', 0.001, 50.0, 'double', 1.0, 0.1, 1),
                      ('kp', -500.0, 500.0, 'double', 10.0, 0.01, 2),
                      ('kd', -500.0, 500.0, 'double', 10.0, 0.01, 2),
                      ('scale',     1,        500,     'int', 10.0, 1.0, 3),
                      ('font_size', 6.0,      128.0,   'double', 24.0, 1.0, 3),
                      ('method', 0, 0, 'combo', 0, 0, 0),
                      ('control', 0, 0, 'combo', 4, 0, 0),
                      ('save_charts', 0, 0, 'checkbox', 0, 0, 0)
                      ]

        conversions = {'system_parameters': 'Parametry symulacji',
                       'initial_conditions': u'Warunki początkowe',
                       'pid_parameters': 'Parametry do kontrolera PID',
                       'display_parameters': 'Parametry wizualne',
                       'l': 'długość', 'g': 'g', 'm': 'm',
                       'omega': 'omega', 'theta': 'theta', 'scale': 'skala',
                       'dt': 'krok czasowy',
                       'tmax': 'czas symulacji', 'kp': 'kp',
                       'kd': 'kd', 'font_size': 'rozmiar czcionki',
                       'method': 'metoda numeryczna',
                       'control': 'metoda sterowania',
                       'save_charts': 'zapisuj wykresy',
                       }

        combobox_map = {'method': ['Isoda'],
                        'control': ['Bez', 'PID', 'PolePlacement',
                                    'Stabilizacja 0', 'Stabilizacja PI_5']
                        }

        checkbox_list = [('mode', 'Obliczanie w chwili', True, 2), ]

        setup_widget_data = SetupWidgetData
        setup_widget_data.label_list = labels
        setup_widget_data.numeric_list = parameters
        setup_widget_data.translations = conversions
        setup_widget_data.combobox_map = combobox_map
        setup_widget_data.checkbox_list = checkbox_list

        # a figure instance to plot on
        figure1 = plt.figure(1)
        canvas1 = FigureCanvas(figure1)
        figure2 = plt.figure(2)
        canvas2 = FigureCanvas(figure2)
        figure3 = plt.figure(3)
        canvas3 = FigureCanvas(figure3)

        list_of_charts = [canvas1, canvas2, canvas3]

        MainWindowBase.__init__(self, data=setup_widget_data,
                                VisualModelType=VPendulumModel,
                                list_of_charts=list_of_charts,
                                window_title=window_name,
                                window_icon=icon_name)


def uncaught_exceptions(ex_type, ex_value, ex_traceback):
    lines = traceback.format_exception(ex_type, ex_value, ex_traceback)
    msg = ''.join(lines)
    logger.error('Uncaught Exception\n%s', msg)


def main():
    sys.excepthook = uncaught_exceptions
    logging.basicConfig(level=logging.INFO)
    logging.getLogger().setLevel(logging.INFO)
    appQt = QtGui.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    appQt.exec_()

if __name__ == '__main__':
    main()
