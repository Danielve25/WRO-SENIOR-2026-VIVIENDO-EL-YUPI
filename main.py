from yupifuncionesbasicas import (
    avance_adelante,
    giro,
    avance_reversa,
    hub,
    wait,
    motor_derecho,
    motor_izquierdo,
    giro_un_motor,
)

hub.imu.reset_heading(0)


giro_un_motor(
    -50, 180, True, False
)  # Gira el robot a 180 grados usando solo el motor derecho


motor_derecho.brake()
motor_izquierdo.brake()

print(
    hub.imu.heading()
)  # Debería imprimir aproximadamente 180 grados, confirmando el giro correctos
