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
    reset_motores,
    segundo_plano,
    motor_garra,
    subir_garra,
    bajar_garra,
    reset_all,
    reset_imu,
)

reset_all()
segundo_plano(reset_motores())
avance_adelante(70, 869, 0)

# escaneo melisimo

motor_derecho.brake()
motor_izquierdo.brake()
