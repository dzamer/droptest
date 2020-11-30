from PyQt5 import QtWidgets, uic
import SerialPortConnection
import ExcelSaveLoad
import pyqtgraph as pg
import sys
import threading
import math


class MainWindow(QtWidgets.QWidget):

    """
    Main class to connect data, Arduino, GUI and calculation for impact force.
    Assumptions:
        * Comunicate with Arduino, DB and GUI,
        * Making calculation for impact force based on Newton's law equations,
        * Display results on graphs,
    """

    readingLines = False
    readingLoop = True
    readedDataFromArduino = []
    clickedButtonData = ""
    fullReadedDate = ""
    fullTimeDate = []
    fullLoadSensorDate = []
    fullUpperSensorDate = []
    fullLowerSensorDate = []
    fullAccelerometerXDate = []
    fullAccelerometerYDate = []
    fullAccelerometerZDate = []
    height = 1.06
    upperSensor = 0.0
    lowerSensor = 0.0
    fallTime = 0.0
    loadMax = 0.0
    acceMax = 0.0
    acceMin = 0.0
    loadInTime = 0.0
    acceInTime = 0.0
    acceLowInTime = 0.0
    brakingTime = 0.0
    averageAcce = 0.0
    velocity = 0.0
    reflectionTime = 0.0
    reflectionDistance = 0.0
    reflectionAcceleration = 0.0
    upperIndex = 0.0
    lowerIndex = 0.0
    wnet = 0.0
    mass = 2.0
    impactForceMethod1 = 0.0
    impactForceMethod2 = 0.0
    counter = 0

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        uic.loadUi('DropTest.ui', self)
        self.setWindowTitle('Drop Test Window')
        self.Arduino = SerialPortConnection.SerialPort()
        self.ExcelOperations = ExcelSaveLoad.ExcelOperation()
        self.statusBar = QtWidgets.QStatusBar()
        # przyciski wszystkie sa włączone po to żeby szło aktualnie programem operować bez arduino
        # navigate buttons
        self.Button_exit.clicked.connect(self.close)
        self.SearchForDevices.clicked.connect(self.Arduino.search_for_devices)
        self.ConnectButton.clicked.connect(self.arduino_connection)
        # buttons to set accelerometer range
        self.Button_range_2G.setDisabled(True)
        # self.Button_range_2G.clicked.connect(self.test("12", 2))
        # self.Button_range_4G.setDisabled(True)
        # self.Button_range_4G.clicked.connect(self.test("13", 2))
        self.Button_range_8G.setDisabled(True)
        self.Button_range_8G.clicked.connect(self.button_clicked_8G)
        self.Button_range_16G.setDisabled(True)
        self.Button_range_16G.clicked.connect(self.button_clicked_16G)
        # buttons to set accelerometer data rate
        self.Button_data_rate_1Hz.setDisabled(True)
        self.Button_data_rate_1Hz.clicked.connect(self.button_clicked_1Hz)
        self.Button_data_rate_10Hz.setDisabled(True)
        self.Button_data_rate_10Hz.clicked.connect(self.button_clicked_10Hz)
        self.Button_data_rate_25Hz.setDisabled(True)
        self.Button_data_rate_25Hz.clicked.connect(self.button_clicked_25Hz)
        self.Button_data_rate_50Hz.setDisabled(True)
        self.Button_data_rate_50Hz.clicked.connect(self.button_clicked_50Hz)
        self.Button_data_rate_100Hz.setDisabled(True)
        self.Button_data_rate_100Hz.clicked.connect(self.button_clicked_100Hz)
        self.Button_data_rate_200Hz.setDisabled(True)
        self.Button_data_rate_200Hz.clicked.connect(self.button_clicked_200Hz)
        self.Button_data_rate_400Hz.setDisabled(True)
        self.Button_data_rate_400Hz.clicked.connect(self.button_clicked_400Hz)
        self.pushButton_data_rate_5KHz_lp.setDisabled(True)
        self.pushButton_data_rate_5KHz_lp.clicked.connect(self.button_clicked_5KHz)
        self.pushButton_data_rate_16KHz_lp.setDisabled(True)
        self.pushButton_data_rate_16KHz_lp.clicked.connect(self.button_clicked_16KHz)
        self.pushButton_data_rate_power_down.setDisabled(True)
        self.pushButton_data_rate_power_down.clicked.connect(self.button_clicked_power_down)
        self.Button_start_test.setDisabled(True)
        self.Button_start_test.clicked.connect(self.button_start)
        # temporary buttons
        # self.Button_readData.setDisabled(True)
        self.Button_readData.clicked.connect(self.plot_generating)
        self.Button_reset.setDisabled(True)
        self.Button_reset.clicked.connect(self.arduino_reading_exit)
        self.update()

    def read_date(self):
        self.Arduino.read_serial()
        if self.Arduino.readedData != "" and self.readingLines is False and \
                self.Arduino.readedData != "Droptest started..":
            self.textBrowser_main.append(self.Arduino.readedData)
        elif self.Arduino.readedData == "Droptest started..":
            self.textBrowser_main.append(self.Arduino.readedData)
            self.readingLines = True
            self.Arduino.waitForReadyReadValue = 50
        elif self.Arduino.readedData != "Droptest started.." and self.Arduino.readedData != "":
            self.readedDataFromArduino = (str(self.Arduino.readedData))
            print(self.readedDataFromArduino)
            temp2 = self.readedDataFromArduino.split(";")
            temp = temp2[0].split(",")
            self.fullTimeDate.append(float(temp[0]))
            self.fullUpperSensorDate.append(float(temp[1]))
            self.fullLowerSensorDate.append(float(temp[2]))
            self.fullAccelerometerXDate.append(float(temp[3]))
            self.fullAccelerometerYDate.append(float(temp[4]))
            self.fullAccelerometerZDate.append(float(temp[5]))

    def read_data_in_loop(self):
        self.readingLoop = True
        while self.readingLoop:
            self.read_date()

    def arduino_reading_exit(self):
        self.readingLoop = False
        self.textBrowser_main.append("Loading sensors data finished.")
        self.Button_readData.setDisabled(False)

    def arduino_connection(self):
            self.Arduino.find_device()
            self.Arduino.open_serial_port()
            thread = threading.Thread(target=self.read_data_in_loop)
            thread.start()
            if self.Arduino.mySerialPort.isOpen():
                self.range_button_enabling()

    def range_button_enabling(self):
        self.Button_range_2G.setDisabled(False)
        self.Button_range_4G.setDisabled(False)
        self.Button_range_8G.setDisabled(False)
        self.Button_range_16G.setDisabled(False)

    def range_button_disabling(self):
        self.Button_range_2G.setDisabled(True)
        self.Button_range_4G.setDisabled(True)
        self.Button_range_8G.setDisabled(True)
        self.Button_range_16G.setDisabled(True)

    def data_rate_button_enabling(self):
        self.Button_data_rate_1Hz.setDisabled(False)
        self.Button_data_rate_10Hz.setDisabled(False)
        self.Button_data_rate_25Hz.setDisabled(False)
        self.Button_data_rate_50Hz.setDisabled(False)
        self.Button_data_rate_100Hz.setDisabled(False)
        self.Button_data_rate_200Hz.setDisabled(False)
        self.Button_data_rate_400Hz.setDisabled(False)
        self.pushButton_data_rate_5KHz_lp.setDisabled(False)
        self.pushButton_data_rate_16KHz_lp.setDisabled(False)
        self.pushButton_data_rate_power_down.setDisabled(False)

    def data_rate_button_disabling(self):
        self.Button_data_rate_1Hz.setDisabled(True)
        self.Button_data_rate_10Hz.setDisabled(True)
        self.Button_data_rate_25Hz.setDisabled(True)
        self.Button_data_rate_50Hz.setDisabled(True)
        self.Button_data_rate_100Hz.setDisabled(True)
        self.Button_data_rate_200Hz.setDisabled(True)
        self.Button_data_rate_400Hz.setDisabled(True)
        self.pushButton_data_rate_5KHz_lp.setDisabled(True)
        self.pushButton_data_rate_16KHz_lp.setDisabled(True)
        self.pushButton_data_rate_power_down.setDisabled(True)

    def button_clicked_2G(self):
            self.clickedButtonData = "12"
            self.data_rate_button_enabling()
            self.textBrowser_main.append("Accelerometer range set to: {0}".format(self.clickedButtonData))
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.range_button_disabling()

    def button_clicked_4G(self):
            self.clickedButtonData = "13"
            self.data_rate_button_enabling()
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.range_button_disabling()

    def button_clicked_8G(self):
            self.clickedButtonData = "14"
            self.data_rate_button_enabling()
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.range_button_disabling()

    def button_clicked_16G(self):
            self.clickedButtonData = "15"
            self.data_rate_button_enabling()
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.range_button_disabling()

    def button_clicked_1Hz(self):
            self.clickedButtonData = "21"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_10Hz(self):
            self.clickedButtonData = "22"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_25Hz(self):
            self.clickedButtonData = "23"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_50Hz(self):
            self.clickedButtonData = "24"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_100Hz(self):
            self.clickedButtonData = "25"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_200Hz(self):
            self.clickedButtonData = "26"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_400Hz(self):
            self.clickedButtonData = "27"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_5KHz(self):
            self.clickedButtonData = "28"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_16KHz(self):
            self.clickedButtonData = "29"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.range_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_clicked_power_down(self):
            self.clickedButtonData = "30"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.data_rate_button_disabling()
            self.Button_start_test.setDisabled(False)

    def button_start(self):
            self.clickedButtonData = "41"
            self.Arduino.dataToWrite = self.clickedButtonData
            self.Arduino.write_serial()
            self.textBrowser_main.append("Loading sensors data...")
            self.Button_reset.setDisabled(False)

    def plot_generating(self):
            self.ExcelOperations.load_from_excel("wyniki", "wyniki1")
            for data in range(0, len(self.ExcelOperations.timeData)):
                self.fullTimeDate.append(self.ExcelOperations.timeData[data])
                self.fullUpperSensorDate.append(self.ExcelOperations.upperSensorData[data])
                self.fullLowerSensorDate.append(self.ExcelOperations.lowerSensorData[data])
                self.fullAccelerometerXDate.append(self.ExcelOperations.accelerometerXData[data])
                self.fullAccelerometerYDate.append(self.ExcelOperations.accelerometerYData[data])
                self.fullAccelerometerZDate.append(self.ExcelOperations.accelerometerZData[data])
            tempTime = self.fullTimeDate[0]
            for var3 in range(0, len(self.fullTimeDate)):
                if self.fullTimeDate[var3] - self.fullTimeDate[0] == 0:
                    self.fullTimeDate[var3] = 0.0
                else:
                    self.fullTimeDate[var3] = (self.fullTimeDate[var3] - tempTime) / 1000
            # setting fall time and max values for load sensor and accelerometer
            self.fall_time()
            self.reflection_travel()
            self.impact_force_method_1_and_2()

            # label info1/2/3
            # self.save_to_excel()
            self.label_info1.setText("Fall time: {0} [s]\n\n"
                                     "Reflection travel: {1} [m]\n\n"
                                     "Reflection time: {2} [s]\n\n"
                                     "Force from method 1: {3} [N]\n\n"
                                     "Force from method 2: {4} [N]".format(self.fallTime,
                                                                           self.reflectionDistance,
                                                                           self.reflectionTime,
                                                                           self.impactForceMethod1,
                                                                           self.impactForceMethod2))

            # start sending data to creating plots
            # accelerometer 3 axies date
            self.plot_accelerometer(self.fullTimeDate, self.fullAccelerometerXDate, 'b', "Acce. X")
            self.plot_accelerometer(self.fullTimeDate, self.fullAccelerometerYDate, 'g', "Acce. Y")
            self.plot_accelerometer(self.fullTimeDate, self.fullAccelerometerZDate, 'r', "Acce. Z")
            # all sensors combined
            tempAcceX = []
            tempAcceY = []
            tempAcceZ = []
            tempLowerSensor = []
            tempUpperSensor = []
            tempTime1 = []
            for var in range(0, len(self.fullTimeDate)):
                if self.fullUpperSensorDate[var] == 1:
                    tempAcceX.append(self.fullAccelerometerXDate[var])
                    tempAcceY.append(self.fullAccelerometerYDate[var])
                    tempAcceZ.append(self.fullAccelerometerZDate[var])
                    tempLowerSensor.append(self.fullLowerSensorDate[var])
                    tempUpperSensor.append(self.fullUpperSensorDate[var])
                    tempTime1.append(self.fullTimeDate[var])
            self.plot_all(tempTime1, tempAcceX, 'b', "Acce. X")
            self.plot_all(tempTime1, tempAcceY, 'g', "Acce. Y")
            self.plot_all(tempTime1, tempAcceZ, 'r', "Acce. Z")
            self.plot_all(tempTime1, tempUpperSensor, 'y', "S. Upper")
            self.plot_all(tempTime1, tempLowerSensor, 'c', "S. Lower")

    def fall_time(self):
        for var in range(0, len(self.fullUpperSensorDate)):
            if self.fullUpperSensorDate[var] == 1:
                self.upperSensor = self.fullTimeDate[var]
                self.upperIndex = var
                break
        for var2 in range(0, len(self.fullLowerSensorDate)):
            if self.fullLowerSensorDate[var2] == 0:
                self.lowerSensor = self.fullTimeDate[var2]
                self.lowerIndex = var2
                break
        self.fallTime = (self.lowerSensor - self.upperSensor)

    def average_acceleration(self):
        temp = 0.0
        count = 0
        for var in range(int(self.upperIndex), int(self.lowerIndex)):
            temp += self.fullAccelerometerXDate[var]
            count += 1
        self.averageAcce = temp / count

    def impact_force_method_1_and_2(self):
        self.average_acceleration()
        self.velocity = math.sqrt(2 * self.averageAcce * self.height)
        self.wnet = 0.5 * self.mass * math.pow(self.velocity, 2)
        self.impactForceMethod1 = self.wnet / self.reflectionDistance
        self.impactForceMethod2 = self.mass * self.reflectionAcceleration

    def reflection_travel(self):
        temp = []
        temp2 = 0.0
        for var in range(0, len(self.fullAccelerometerXDate)):
            if self.fullAccelerometerXDate[var] < 9.0:
                temp.append(self.fullTimeDate[var])
                print(self.fullTimeDate[var])
                temp2 += self.fullAccelerometerXDate[var]

        for var2 in range(1, len(self.fullAccelerometerXDate)):
            if self.fullAccelerometerXDate[var2] < 9.0 and self.fullAccelerometerXDate[var2] < \
                    self.fullAccelerometerXDate[var2 - 1]:
                tempTime = self.fullTimeDate[var2] - self.fullTimeDate[var2 - 1]
                self.reflectionDistance += (abs(tempTime) * abs(self.fullAccelerometerXDate[var2]))

        self.reflectionTime = (temp[len(temp) - 1] - temp[0])
        self.reflectionAcceleration = 2.0 * (
            math.sqrt(2.0 * (temp2/len(temp)) * abs(self.reflectionDistance))) / self.reflectionTime

    def plot_all(self, time, axis, color, name):
        pen = pg.mkPen(color=color, width=2)
        self.widget_plot_all.setBackground('w')
        self.widget_plot_all.addLegend()
        self.widget_plot_all.setLabel("left", "Acceleration [m/s2]")
        self.widget_plot_all.setLabel("bottom", "Time [s]")
        self.widget_plot_all.showGrid(x=True, y=True)
        self.widget_plot_all.plot(time, axis, pen=pen, name=name)

    def plot_accelerometer(self, time, axis, color, name):
        pen = pg.mkPen(color=color, width=2)
        self.widget_plot_accelerometer.setBackground('w')
        self.widget_plot_accelerometer.addLegend()
        self.widget_plot_accelerometer.setLabel("left", "Acceleration [m/s2]")
        self.widget_plot_accelerometer.setLabel("bottom", "Time [s]")
        self.widget_plot_accelerometer.showGrid(x=True, y=True)
        self.widget_plot_accelerometer.plot(time, axis, pen=pen, name=name)


def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
