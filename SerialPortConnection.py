from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice


class SerialPort:
    mySerialPort = QSerialPort()
    devicePortInfo = None
    readedData = ""
    dataToWrite = ""
    deviceIsFound = False
    deviceIsConnected = False
    waitForReadyReadValue = 200
    productIdentifier = 29987  # product ID to be set for device if other will be used (use serachForDevices if needed)
    vendorIdentifier = 6790  # vendor ID to be set for device if other will be used (use serachForDevices if needed)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.search_for_devices()

    # use serachForDevices if you need to get vendor and product ID for findDevice function
    def search_for_devices(self):
        device = QSerialPortInfo
        availablePort = device.availablePorts()

        i = 0
        for portInfo in availablePort:
            print("Device: {0}, product ID: {1}, vendor ID: {2}, Port name: {3}".format(i,
                                                                                        portInfo.productIdentifier(),
                                                                                        portInfo.vendorIdentifier(),
                                                                                        portInfo.portName()))
            i += 1

    def find_device(self):
        info = QSerialPortInfo
        availablePorts = info.availablePorts()

        for portInfo in availablePorts:
            print(type(portInfo.productIdentifier()))
            if portInfo.productIdentifier() == self.productIdentifier and portInfo.vendorIdentifier() \
                    == self.vendorIdentifier:
                self.devicePortInfo = portInfo
                self.deviceIsFound = True
                print("Device found at port: {0}".format(portInfo.portName))
                break

    def open_serial_port(self):
        try:
            self.mySerialPort.setPortName(str(self.devicePortInfo.portName()))
            self.mySerialPort.open(QIODevice.ReadWrite)
            self.mySerialPort.setBaudRate(2000000)
            self.mySerialPort.setDataBits(self.mySerialPort.Data8)
            print("Opening port {0}".format(self.mySerialPort.portName()))
            self.mySerialPort.setDataTerminalReady(True)
            self.deviceIsConnected = True
            print(self.mySerialPort.isOpen())
            print(self.mySerialPort.portName())
            self.mySerialPort.startTransaction()
            print(self.mySerialPort.isDataTerminalReady())
            print(self.mySerialPort.isReadable())
            print(self.mySerialPort.isWritable())
            return self.mySerialPort
        except Exception as e:
            print("open_serial_port: ", e)
            return None

    def read_serial(self):
        if self.deviceIsConnected:
            self.mySerialPort.waitForReadyRead(self.waitForReadyReadValue)
            temp = str(self.mySerialPort.readLineData(1024))
            temp2 = temp.split("'")
            temp3 = temp2[1].split("\\r")
            self.readedData = temp3[0]
        else:
            print("Device not connected")
            return

    def write_serial(self):
        if self.deviceIsConnected:
            dataEncoded = self.dataToWrite.encode()
            self.mySerialPort.write(dataEncoded)
            print("You enter {0} to serial.".format(self.dataToWrite))
        else:
            print("Device not connected")