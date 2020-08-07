from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
import sys


class SerialPort(QSerialPort):


    def open_serial_port(self):
        try:
            serial_port = QSerialPort()
            serial_port.setPortName(self)
            serial_port.setBaudRate(QSerialPort.Baud9600, QSerialPort.AllDirections)
            serial_port.setParity(QSerialPort.NoParity)
            serial_port.setStopBits(QSerialPort.OneStop)
            serial_port.setDataBits(QSerialPort.Data8)
            serial_port.setFlowControl(QSerialPort.NoFlowControl)
            serial_port.open(QSerialPort.ReadWrite)
            return serial_port
        except Exception as e:
            print("open_serial_port", e)
            return None


    def get_serial_port(self):
        # if sys.platform.startswith('linux'):
        #     serial_ports = glob.glob('/dev/athome*')
        # else:
        info_list = QSerialPortInfo()
        serial_list = info_list.availablePorts()
        serial_ports = [port.portName() for port in serial_list]
        return serial_ports


    def read_serial(self):
        data = self.read()
        return data


    def write_serial(self, data):
        return self.write(b'data')
