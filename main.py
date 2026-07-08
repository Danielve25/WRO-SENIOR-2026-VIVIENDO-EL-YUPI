from yupifuncionesbasicas import (
    avance_adelante,
    bajar_barrera,
    giro,
    avance_reversa,
    hub,
    wait,
    motor_derecho,
    motor_izquierdo,
    giro_un_motor,
    reset_motores,
    segundo_plano,
    motor_garra,
    subir_garra,
    bajar_garra,
    reset_all,
    reset_imu,
    color_sensor,
    subir_garra,
    seguir_linea_dc,
    soltar_cubo,
    subir_barrera,
)

reset_all()
segundo_plano(reset_motores())

avance_adelante(50, 1000, 0)

# ---------------codigo cubos-----------------

# soltar_cubo()
# bajar_barrera(180)
# wait(1000)
# subir_barrera(180)


motor_derecho.brake()
motor_izquierdo.brake()
