#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 2018, Wojciech Rembelski
# -----------------------------------------------------------------------------

from __future__ import division, print_function, absolute_import
from vispy import app
import numpy as np
import logging
import traceback
from PyQt4 import QtGui, QtCore

logger = logging.getLogger(__name__)


class VisualModel(app.Canvas):

    def __init__(self):

        app.Canvas.__init__(self, title='Symulacja modelu', size=(800, 800))

        # Some initialization constants that won't change
        self.center = np.asarray((500, 250))
        self.visuals = []

        # Set up a timer to update the image and give a real-time rendering
        self._timer = app.Timer('auto', connect=self.on_timer, start=True)
        self.show()

    def on_draw(self, ev):
        self.context.set_clear_color((0, 0, 0, 1))
        self.context.set_viewport(0, 0, *self.physical_size)
        self.context.clear()
        for vis in self.visuals:
            vis.draw()

    def on_resize(self, event):
        # Set canvas viewport and reconfigure visual transforms to match.
        vp = (0, 0, self.physical_size[0], self.physical_size[1])
        self.context.set_viewport(*vp)

        for vis in self.visuals:
            vis.transforms.configure(canvas=self, viewport=vp)


class SetupWidgetData():
    label_list = []
    numeric_list = []
    checkbox_list = []
    combobox_map = []
    translations = dict()
    pass


class Paramlist(object):

    def __init__(self, data):
        self.params = data.numeric_list
        self.props = dict()
        for nameV, minV, maxV, typeV, iniV, stepV, labelV in data.numeric_list:
            if typeV == "combo":
                self.props[nameV] = data.combobox_map[nameV][iniV]
            else:
                self.props[nameV] = iniV


class SetupWidget(QtGui.QWidget):

    changed_parameter_sig = QtCore.pyqtSignal(Paramlist)

    def __init__(self, parent, data):
        super(SetupWidget, self).__init__(parent)

        self.data = data
        self.param = Paramlist(data)

        self.groupbox_list = []

        for group_id, group_name in self.data.label_list:
            self.groupbox_list.append(QtGui.QGroupBox(
                data.translations[group_name]))

        self.splitter = QtGui.QSplitter(QtCore.Qt.Vertical)

        plist = []
        self.psets = []

        param_boxes_layout = []
        for _ in range(len(self.data.label_list)):
            param_boxes_layout.append(QtGui.QGridLayout())

        self.label_map = dict()
        self.combo_label_map = dict()

        for nameV, minV, maxV, typeV, iniV, stepV, labelV in self.param.params:
            # Create Labels for each element
            plist.append(QtGui.QLabel(unicode(self.data.translations[nameV],
                                              'utf-8')))

            self.label_map[len(plist)-1] = labelV

            if typeV == 'double':
                self.psets.append(QtGui.QDoubleSpinBox())
                self.psets[-1].setDecimals(3)
                self.psets[-1].setSingleStep(stepV)
            elif typeV == 'int':
                self.psets.append(QtGui.QSpinBox())
            elif typeV == 'combo':
                self.psets.append(QtGui.QComboBox())
                self.psets[-1].addItems(self.data.combobox_map[nameV])
                self.psets[-1].setCurrentIndex(iniV)
            elif typeV == 'checkbox':
                self.psets.append(QtGui.QCheckBox(self.data.translations[nameV]))
                self.psets[-1].setChecked(bool(iniV))

            # Set min, max, and initial values
            if typeV != 'combo' and typeV != 'checkbox':
                self.psets[-1].setMaximum(maxV)
                self.psets[-1].setMinimum(minV)
                self.psets[-1].setValue(iniV)

        for pos in range(len(plist)):
            pidx = int(self.label_map[pos])
            if type(self.psets[pos]) is QtGui.QCheckBox:
                param_boxes_layout[pidx].addWidget(self.psets[pos], pos + pidx, 0)
            else:
                param_boxes_layout[pidx].addWidget(plist[pos], pos + pidx, 0)
                param_boxes_layout[pidx].addWidget(self.psets[pos], pos + pidx, 1)

            if type(self.psets[pos]) is QtGui.QComboBox:
                self.psets[pos].currentIndexChanged.connect(self.update_params)
            elif type(self.psets[pos]) is QtGui.QCheckBox:
                self.psets[pos].toggled.connect(self.update_params)
            else:
                self.psets[pos].valueChanged.connect(self.update_params)

        for groupbox, layout in zip(self.groupbox_list, param_boxes_layout):
            groupbox.setLayout(layout)

        for groupbox in self.groupbox_list:
            self.splitter.addWidget(groupbox)

        vbox = QtGui.QVBoxLayout()
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.splitter)
        hbox.addStretch(5.0)
        vbox.addLayout(hbox)
        vbox.addStretch(1.0)

        self.setLayout(vbox)

    def update_params(self, option):
        """When the system parameters change, get the state and emit it."""

        keys = map(lambda x: x[0], self.param.params)
        for pos, nameV in enumerate(keys):
            if isinstance(self.psets[pos], QtGui.QComboBox):
                self.param.props[nameV] = \
                    self.data.combobox_map[nameV][self.psets[pos].currentIndex()]
            elif isinstance(self.psets[pos], QtGui.QCheckBox):
                self.param.props[nameV] = self.psets[pos].isChecked()
            else:
                self.param.props[nameV] = self.psets[pos].value()

        self.changed_parameter_sig.emit(self.param)


class MainWindowBase(QtGui.QMainWindow):

    def __init__(self, data, list_of_charts, VisualModelType,
                 window_title="PhysicalSimulation", window_icon=None):
        QtGui.QMainWindow.__init__(self)

        # self.resize(1366, 768)
        self.setFixedSize(1366, 708)
        # self.showFullScreen()
        if window_icon is not None:
            self.setWindowIcon(QtGui.QIcon(window_icon))
        self.setWindowTitle(window_title)

        self.parameter_object = SetupWidget(self, data)
        self.parameter_object.param = (self.parameter_object.param)
        self.parameter_object.changed_parameter_sig.connect(self.update_view)

        self.view_box = VisualModelType(list_of_charts, **self.parameter_object.param.props)

        self.view_box.create_native()
        self.view_box.native.setParent(self)

        splitter = QtGui.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.parameter_object)
        splitter.addWidget(self.view_box.native)

        if list_of_charts:
            splitterV = QtGui.QSplitter(QtCore.Qt.Vertical)
            for chart in list_of_charts:
                splitterV.addWidget(chart)
            splitter.addWidget(splitterV)
            splitter.setStretchFactor(2, 2)

        self.setCentralWidget(splitter)

        # zamykanie programu
        exitAction = QtGui.QAction(QtGui.QIcon('exit.png'), '&Exit', self)
        exitAction.setShortcut('Esc')
        # exitAction.setShortcut('Ctrl+C')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(QtGui.qApp.quit)
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(exitAction)


    def update_view(self, param):
        self.view_box.reset_parms(**param.props)


def uncaught_exceptions(ex_type, ex_value, ex_traceback):
    lines = traceback.format_exception(ex_type, ex_value, ex_traceback)
    msg = ''.join(lines)
    logger.error('Uncaught Exception\n%s', msg)
