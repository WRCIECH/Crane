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
from VehicleKinematicModel import VehicleKinematicModel

import sys
sys.path.insert(0, '../')
try:
    from VisualModel import VisualModel, MainWindowBase, SetupWidgetData
except:
    raise


logger = logging.getLogger(__name__)


class CraneModel(VisualModel):

    def __init__(self, list_of_charts, theta, fi, kp, kd,
                 l, scale, x, y, method, control, save_charts,
                 dt, tmax, font_size):

        VisualModel.__init__(self)

        self.center = [400, 400]
        self.model = VehicleKinematicModel(list_of_charts)
        self.list_of_charts = list_of_charts

        self._set_up_system(
            l=l, theta=theta, fi=fi, x=x, y=y, kp=kp,
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

        self.rear_wheelsL = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='yellow')
        self.rear_wheelsL.transform = transforms.MatrixTransform()
        self.rear_wheelsL.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.rear_wheelsL.transform.translate([0.0, -self.scale*0.5*4.0])
        self.rear_wheelsL.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.rear_wheelsL.transform.translate([self.model.x, self.model.y])
        self.rear_wheelsL.transform.translate(self.center)

        self.rear_wheelsR = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='yellow')
        self.rear_wheelsR.transform = transforms.MatrixTransform()
        self.rear_wheelsR.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.rear_wheelsR.transform.translate([0.0, self.scale*0.5*4.0])
        self.rear_wheelsR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.rear_wheelsR.transform.translate([self.model.x, self.model.y])
        self.rear_wheelsR.transform.translate(self.center)

        self.front_wheelsL = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='pink')
        self.front_wheelsL.transform = transforms.MatrixTransform()
        self.front_wheelsL.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.front_wheelsL.transform.rotate(np.rad2deg(-self.model.fi), (0, 0, 1))
        self.front_wheelsL.transform.translate([self.scale*l, -self.scale*0.5*4.0])
        self.front_wheelsL.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.front_wheelsL.transform.translate([self.model.x, self.model.y])
        self.front_wheelsL.transform.translate(self.center)

        self.front_wheelsR = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='pink')
        self.front_wheelsR.transform = transforms.MatrixTransform()
        self.front_wheelsR.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.front_wheelsR.transform.rotate(np.rad2deg(-self.model.fi), (0, 0, 1))
        self.front_wheelsR.transform.translate([self.scale*l, self.scale*0.5*4.0])
        self.front_wheelsR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.front_wheelsR.transform.translate([self.model.x, self.model.y])
        self.front_wheelsR.transform.translate(self.center)

        self.vehicle_body = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='green')
        self.vehicle_body.transform = transforms.MatrixTransform()
        self.vehicle_body.transform.translate([0.5, 0.0])
        self.vehicle_body.transform.scale((self.scale*self.model.l, self.scale, 1))
        self.vehicle_body.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_body.transform.translate([self.model.x, self.model.y])
        self.vehicle_body.transform.translate(self.center)

        self.vehicle_bodyF = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='red')
        self.vehicle_bodyF.transform = transforms.MatrixTransform()
        self.vehicle_bodyF.transform.scale((self.scale*0.25, self.scale*4.0, 1))
        self.vehicle_bodyF.transform.translate([l*self.scale, 0.0])
        self.vehicle_bodyF.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_bodyF.transform.translate([self.model.x, self.model.y])
        self.vehicle_bodyF.transform.translate(self.center)

        self.vehicle_bodyR = visuals.BoxVisual(width=1.0, height=1.0, depth=1.0,
                                              color='blue')
        self.vehicle_bodyR.transform = transforms.MatrixTransform()
        self.vehicle_bodyR.transform.scale((self.scale*0.25, self.scale*4.0, 1))
        self.vehicle_bodyR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_bodyR.transform.translate([self.model.x, self.model.y])
        self.vehicle_bodyR.transform.translate(self.center)

        # Append all the visuals
        self.visuals.append(self.vehicle_body)
        self.visuals.append(self.vehicle_bodyF)
        self.visuals.append(self.vehicle_bodyR)
        self.visuals.append(self.rear_wheelsL)
        self.visuals.append(self.rear_wheelsR)
        self.visuals.append(self.front_wheelsL)
        self.visuals.append(self.front_wheelsR)
        self.visuals.append(self.text)

    def on_timer(self, ev):
        millis_passed = int(100 * (self.model.t % 1))
        sec_passed = int(self.model.t % 60)
        min_passed = int(self.model.t // 60)

        self.rear_wheelsL.transform.reset()
        self.rear_wheelsL.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.rear_wheelsL.transform.translate([0.0, -self.scale*0.5*4.0])
        self.rear_wheelsL.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.rear_wheelsL.transform.translate([self.model.x, self.model.y])
        self.rear_wheelsL.transform.translate(self.center)

        self.rear_wheelsR.transform.reset()
        self.rear_wheelsR.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.rear_wheelsR.transform.translate([0.0, self.scale*0.5*4.0])
        self.rear_wheelsR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.rear_wheelsR.transform.translate([self.model.x, self.model.y])
        self.rear_wheelsR.transform.translate(self.center)

        self.front_wheelsL.transform.reset()
        self.front_wheelsL.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.front_wheelsL.transform.rotate(np.rad2deg(-self.model.fi), (0, 0, 1))
        self.front_wheelsL.transform.translate([self.scale*self.model.l, -self.scale*0.5*4.0])
        self.front_wheelsL.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.front_wheelsL.transform.translate([self.model.x, self.model.y])
        self.front_wheelsL.transform.translate(self.center)

        self.front_wheelsR.transform.reset()
        self.front_wheelsR.transform.scale((self.scale*2.0, self.scale*0.5, 1))
        self.front_wheelsR.transform.rotate(np.rad2deg(-self.model.fi), (0, 0, 1))
        self.front_wheelsR.transform.translate([self.scale*self.model.l, self.scale*0.5*4.0])
        self.front_wheelsR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.front_wheelsR.transform.translate([self.model.x, self.model.y])
        self.front_wheelsR.transform.translate(self.center)

        # Apply the necessary transformations to the vehicle_body
        self.vehicle_body.transform.reset()
        self.vehicle_body.transform.translate([0.5, 0.0])
        self.vehicle_body.transform.scale((self.scale*self.model.l, self.scale, 1))
        self.vehicle_body.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_body.transform.translate([self.model.x, self.model.y])
        self.vehicle_body.transform.translate(self.center)

        self.vehicle_bodyF.transform.reset()
        self.vehicle_bodyF.transform.scale((self.scale*0.25, self.scale*4.0, 1))
        self.vehicle_bodyF.transform.translate([self.model.l*self.scale, 0.0])
        self.vehicle_bodyF.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_bodyF.transform.translate([self.model.x, self.model.y])
        self.vehicle_bodyF.transform.translate(self.center)

        self.vehicle_bodyR.transform.reset()
        self.vehicle_bodyR.transform = transforms.MatrixTransform()
        self.vehicle_bodyR.transform.scale((self.scale*0.25, self.scale*4.0, 1))
        self.vehicle_bodyR.transform.rotate(np.rad2deg(-self.model.theta), (0, 0, 1))
        self.vehicle_bodyR.transform.translate([self.model.x, self.model.y])
        self.vehicle_bodyR.transform.translate(self.center)

        # Update the timer with how long it's been
        self.text.text = '{:0>2d}:{:0>2d}.{:0>2d}'.format(min_passed,
                                                          sec_passed,
                                                          millis_passed)

        self.update()
        self.model.do_time_step()

    def reset_parms(self, l, kp, kd, theta, fi, x, y, scale, method,
                    control, save_charts, dt, tmax, font_size):

        self._set_up_system(
            l=l, theta=theta, fi=fi, x=x, y=y, kp=kp,
            kd=kd, scale=scale, method=method, control=control,
            save_charts=save_charts, dt=dt, tmax=tmax, font_size=font_size
        )

    def _set_up_system(self, l, kp, kd, theta,
                       fi, x, y, scale, method, control, save_charts,
                       dt, tmax, font_size):

        self.model._set_up_system(l=l, kp=kp, kd=kd, theta=theta,
                                  x=x, y=y, fi=fi, method=method,
                                  control=control, save_charts=save_charts,
                                  dt=dt, tmax=tmax)

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
        window_name = 'Crane simulation'

        labels = [
            (0, 'system_parameters'),
            (1, 'initial_conditions'),
            (2, 'pid_parameters'),
            (3, 'display_parameters')]

        parameters = [('l',         0.1,      10.0,    'double', 5.0, 0.1, 0),
                      ('dt', 0.001,    1.0,     'double', 1/60, 0.001, 0),
                      ('tmax', 1.0,    100.0,     'double', 50.0, 1.0, 0),
                      ('theta', -np.pi, np.pi, 'double', 0.0, 0.01, 1),
                      ('fi', -np.pi/2, np.pi/2, 'double', 0.0, 0.01, 1),
                      ('x', -50.0, 50.0, 'double', 0.0, 0.1, 1),
                      ('y', -50.0, 50.0, 'double', 0.1, 0.1, 1),
                      ('kp', 0.0, 500.0, 'double', 1.0, 0.01, 2),
                      ('kd', 0.0, 500.0, 'double', 1.0, 0.01, 2),
                      ('scale',     5,        500,     'int',    10, 1.0, 3),
                      ('font_size', 6.0,      128.0,   'double', 24.0, 1.0, 3),
                      ('method', 0, 0, 'combo', 0, 0, 0),
                      ('control', 0, 0, 'combo', 0, 0, 0),
                      ('save_charts', 0, 0, 'checkbox', 0, 0, 0)
                      ]

        conversions = {'system_parameters': 'Parametry symulacji',
                       'initial_conditions': u'Warunki początkowe',
                       'pid_parameters': 'Parametry do kontrolera PID',
                       'display_parameters': 'Parametry wizualne',
                       'l': 'długość', 'theta': 'theta', 'fi': 'fi',
                       'scale': 'skala', 'x': 'x', 'y': 'y',
                       'dt': 'krok czasowy', 'tmax': 'czas symulacji',
                       'kp': 'kp', 'kd': 'kd', 'font_size': 'rozmiar czcionki',
                       'method': 'metoda numeryczna',
                       'control': 'metoda sterowania',
                       'save_charts': 'zapisuj wykresy',
                       }

        combobox_map = {'method': ['Isoda'],
                        'control': ['Bez', 'PID', 'Fazowe']
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

        list_of_charts = [canvas1, canvas2]

        MainWindowBase.__init__(self, data=setup_widget_data,
                                VisualModelType=CraneModel,
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
