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

from PyQt5.QtWidgets import QVBoxLayout, QWidget, QApplication, QMainWindow, QAction
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QKeySequence

try:
    import pyqtgraph
except ImportError:
    os.system('git clone --recursive https://github.com/pyqtgraph/pyqtgraph-core pyqtgraph')
    import pyqtgraph

from pyqtgraph import PlotWidget, mkPen, InfiniteLine


if len(sys.argv) > 1:
    SER_PORT = sys.argv[1]
else:
    SER_PORT = glob.glob('/dev/serial/by-id/usb-*Black_Magic_Probe*-if02')[0]
    print('Selected port', SER_PORT)

SER_BAUDRATE = 115200


# Borrowed from the UAVCAN GUI Tool
def add_crosshair(plot, render_measurements, color=Qt.gray):
    pen = mkPen(color=QColor(color), width=1)
    vline = InfiniteLine(angle=90, movable=False, pen=pen)
    hline = InfiniteLine(angle=0, movable=False, pen=pen)

    plot.addItem(vline, ignoreBounds=True)
    plot.addItem(hline, ignoreBounds=True)

    current_coordinates = None
    reference_coordinates = None

    def do_render():
        render_measurements(current_coordinates, reference_coordinates)

    def update(pos):
        nonlocal current_coordinates
        if plot.sceneBoundingRect().contains(pos):
            mouse_point = plot.getViewBox().mapSceneToView(pos)
            current_coordinates = mouse_point.x(), mouse_point.y()
            vline.setPos(mouse_point.x())
            hline.setPos(mouse_point.y())
            do_render()

    def set_reference(ev):
        nonlocal reference_coordinates
        if ev.button() == Qt.LeftButton and current_coordinates is not None:
            reference_coordinates = current_coordinates
            do_render()

    plot.scene().sigMouseMoved.connect(update)
    plot.scene().sigMouseClicked.connect(set_reference)


class RealtimePlotWidget(QWidget):
    COLORS = [Qt.red, Qt.blue, Qt.green, Qt.magenta, Qt.cyan,
              Qt.darkRed, Qt.darkBlue, Qt.darkGreen, Qt.darkYellow, Qt.gray]

    def __init__(self, display_measurements, parent):
        super(RealtimePlotWidget, self).__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)              # This is required to stop background timers!
        self._plot_widget = PlotWidget()
        self._plot_widget.setBackground((0, 0, 0))
        self._legend = self._plot_widget.addLegend()
        self._plot_widget.showButtons()
        self._plot_widget.enableAutoRange()
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        vbox = QVBoxLayout(self)
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._update_timer = QTimer(self)
        self._update_timer.setSingleShot(False)
        self._update_timer.timeout.connect(self._update)
        self._update_timer.start(200)

        self._color_index = 0
        self._curves = {}

        # Crosshair
        def _render_measurements(cur, ref):
            text = 'time %.6f sec,  y %.6f' % cur
            if ref is None:
                return text
            dt = cur[0] - ref[0]
            dy = cur[1] - ref[1]
            if abs(dt) > 1e-12:
                freq = '%.6f' % abs(1 / dt)
            else:
                freq = 'inf'
            display_measurements(text + ';' + ' ' * 4 + 'dt %.6f sec,  freq %s Hz,  dy %.6f' % (dt, freq, dy))

        display_measurements('Hover to sample Time/Y, click to set new reference')
        add_crosshair(self._plot_widget, _render_measurements)

    def add_curve(self, curve_id, curve_name, data_x=[], data_y=[]):
        color = QColor(self.COLORS[self._color_index % len(self.COLORS)])
        self._color_index += 1
        pen = mkPen(color, width=1)
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x)
        data_y = numpy.array(data_y)
        self._curves[curve_id] = {'data': (data_x, data_y), 'plot': plot}

    def reset(self):
        for curve in self._curves.keys():
            self._plot_widget.removeItem(self._curves[curve]['plot'])

        self._curves = {}
        self._color_index = 0

        self._plot_widget.enableAutoRange()

        self._legend.scene().removeItem(self._legend)
        self._legend = self._plot_widget.addLegend()

    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        old_x, old_y = curve['data']
        curve['data'] = numpy.append(old_x, x), numpy.append(old_y, y)

    def _update(self):
        for curve in self._curves.values():
            if len(curve['data'][0]):
                curve['plot'].setData(*curve['data'])


class Window(QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setWindowTitle('Serial Plot')

        self.statusBar().show()

        self._plot = RealtimePlotWidget(self.statusBar().showMessage, self)

        # Actions menu
        clear_action = QAction('&Clear', self)
        clear_action.setShortcut(QKeySequence('Ctrl+Shift+C'))
        clear_action.triggered.connect(self._plot.reset)

        actions_menu = self.menuBar().addMenu('&Actions')
        actions_menu.addAction(clear_action)

        # Layout
        self.setCentralWidget(self._plot)

    @property
    def plot(self):
        return self._plot


class SerialReader:
    def __init__(self, port, baudrate, timeout=None, value_prefix='$'):
        self._value_prefix = value_prefix
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, writeTimeout=timeout)

    def poll(self, value_handler, raw_handler):
        line = self._port.readline().decode()
        if not line.startswith(self._value_prefix):
            raw_handler(line)
        else:
            items = eval(line[len(self._value_prefix):])
            if items and len(items) > 1:
                timestamp, items = items[0], items[1:]
                value_handler(timestamp, map(float, items))

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
            window.plot.update_values(i, [x], [val])
        except KeyError:
            window.plot.add_curve(i, str(i), [x], [val])


app = QApplication(sys.argv)

initial_timestamp = time.time()

window = Window()

reader = SerialReader(SER_PORT, SER_BAUDRATE)

cli = CLIInputReader(lambda line: reader._port.write((line + '\r\n').encode()))

threading.Thread(target=reader.run, args=(value_handler, lambda s: print(s.rstrip())), daemon=True).start()

window.show()
exit(app.exec_())
