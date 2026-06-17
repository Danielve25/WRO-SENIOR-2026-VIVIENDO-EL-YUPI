from pybricks.parameters import Stop
from yupifuncionesbasicas import (
    avance_adelante,
    giro,
    avance_reversa,
    hub,
    wait,
    motor_derecho,
    motor_izquierdo,
    giro_un_motor,
    motor_barrera,
    reset_todo,
)

reset_todo()

motor_derecho.brake()
motor_izquierdo.brake()
