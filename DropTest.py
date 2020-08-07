from PyQt5 import QtWidgets, uic
from pyqtgraph import PlotWidget, plot
import numpy as np
import pyqtgraph as pg
import sys

class MainWindow(QtWidgets.QWidget):


    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        uic.loadUi('DropTest.ui', self)

    def load_sensor_enter_value(self):
        if self.Button_load_sensor_enter_value.clicked:
            print(self.load_sensor_line_edit())
        return self.Button_load_sensor_enter_value.clicked.connect(self.load_sensor_line_edit)

    def load_sensor_line_edit(self):
        return self.lineEdit_load_sensor.text()

    def load_sensor_calibration(self):
        self.Button_load_sensor_calibration.clicked.connect(1)

    def load_sensor_load_value(self):
        self.Button_load_sensor_load_value.clicked.connect(2)

    def plot_all(self, time, axis_x, axis_y, axis_z, load_sensor_value, upper_slotted, lower_slotted):
        self.widget_plot_all.setBackgroudn('w')
        self.widget_plot_all.plot(time, axis_x, axis_y, axis_z, load_sensor_value, upper_slotted, lower_slotted)

    def plot_accelerometer(self, time, axis_x, axis_y, axis_z):
        self.widget_plot_accelerometer.setBackground('w')
        self.widget_plot_accelerometer.plot(time, axis_x, axis_y, axis_z)

    def plot_load_sensor(self, time, value):
        self.widget_plot_load_sensor.setBackground('w')
        self.widget_plot_load_sensor.plot(time, value)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
