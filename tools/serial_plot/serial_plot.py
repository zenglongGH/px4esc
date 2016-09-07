#!/usr/bin/env python3
#
# Pavel Kirienko, 2016 <pavel.kirienko@gmail.com>
#
# This is a quick hack that allows to plot values from serial port and at the same time have access to CLI.
# This script may be superseded with Zubax Toolbox at some point.
#

import numpy
import os
import sys
import threading
import time
import serial
import glob

from PyQt5.QtWidgets import QVBoxLayout, QWidget, QApplication
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor

try:
    from pyqtgraph import PlotWidget, mkPen
except ImportError:
    os.system('git clone --recursive https://github.com/pyqtgraph/pyqtgraph-core pyqtgraph')
    from pyqtgraph import PlotWidget, mkPen


if len(sys.argv) > 1:
    SER_PORT = sys.argv[1]
else:
    SER_PORT = glob.glob('/dev/serial/by-id/usb-*Black_Magic_Probe*-if02')[0]
    print('Selected port', SER_PORT)

SER_BAUDRATE = 921600


class RealtimePlotWidget(QWidget):
    COLORS = [Qt.red, Qt.blue, Qt.green, Qt.magenta, Qt.cyan,
              Qt.darkRed, Qt.darkBlue, Qt.darkGreen, Qt.darkYellow, Qt.gray]

    def __init__(self, parent=None):
        super(RealtimePlotWidget, self).__init__(parent)
        self._plot_widget = PlotWidget()
        self._plot_widget.setBackground((0, 0, 0))
        self._plot_widget.addLegend()
        self._plot_widget.showButtons()
        self._plot_widget.enableAutoRange()
        self._plot_widget.showGrid(x=True, y=True, alpha=0.2)
        vbox = QVBoxLayout(self)
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

    def add_curve(self, curve_id, curve_name, data_x=[], data_y=[]):
        color = QColor(self.COLORS[self._color_index % len(self.COLORS)])
        self._color_index += 1
        pen = mkPen(color, width=1)
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x)
        data_y = numpy.array(data_y)
        self._curves[curve_id] = {'x': data_x, 'y': data_y, 'plot': plot}

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._plot_widget.removeItem(self._curves[curve_id]['plot'])
            del self._curves[curve_id]

    def set_x_range(self, left, right):
        self._plot_widget.setRange(xRange=(left, right))

    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        curve['x'] = numpy.append(curve['x'], x)
        curve['y'] = numpy.append(curve['y'], y)

    def redraw(self):
        for curve in self._curves.values():
            if len(curve['x']):
                curve['plot'].setData(curve['x'], curve['y'])

    def lazy_redraw(self, period):
        timestamp = time.time()
        if not hasattr(self, '_prev_lazy_redraw'):
            self._prev_lazy_redraw = 0.0
        if timestamp - self._prev_lazy_redraw > period:
            self._prev_lazy_redraw = timestamp
            self.redraw()


class SerialReader:
    def __init__(self, port, baudrate, timeout=None, value_prefix='$'):
        self._value_prefix = value_prefix
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)
        self._x = 0

    def poll(self, value_handler, raw_handler):
        line = self._port.readline().decode()
        if not line.startswith(self._value_prefix):
            raw_handler(line)
        else:
            self._x += 1
            items = eval(line[len(self._value_prefix):])
            if items:
                value_handler(self._x, map(float, items))

    def run(self, value_handler, raw_handler):
        while True:
            try:
                self.poll(value_handler, raw_handler)
            except Exception as ex:
                print('Serial poll failed:', ex)


class CLIInputReader(threading.Thread):
    def __init__(self, on_line_received):
        super(CLIInputReader, self).__init__(name='CLIInputReader', daemon=True)
        self.on_line_received = on_line_received
        self.start()

    def run(self):
        while True:
            try:
                line = input()
                self.on_line_received(line)
            except Exception as ex:
                print('CLI input poll failed:', ex)


def value_handler(x, values):
    for i, val in enumerate(values):
        try:
            plot.update_values(i, [x], [val])
        except KeyError:
            plot.add_curve(i, str(i), [x], [val])
    plot.lazy_redraw(0.2)


app = QApplication(sys.argv)

initial_timestamp = time.time()

plot = RealtimePlotWidget()

reader = SerialReader(SER_PORT, SER_BAUDRATE)

cli = CLIInputReader(lambda line: reader._port.write((line + '\r\n').encode()))

threading.Thread(target=reader.run, args=(value_handler, lambda s: print(s.rstrip())), daemon=True).start()

plot.redraw()
plot.show()
exit(app.exec_())
