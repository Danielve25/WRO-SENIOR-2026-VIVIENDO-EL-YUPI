from pybricks.parameters import Port
from pybricks.iodevices import UARTDevice

# power_pin=1 activa el voltaje directo de la batería en el Pin 1 (M1)
# El Pin 2 (M2) actuará como ruta de retorno / tierra
sensor = UARTDevice(Port.A, baudrate=115200, power_pin=1)
