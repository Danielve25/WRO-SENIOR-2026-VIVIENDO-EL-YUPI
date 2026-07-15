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
    golpear_pared,
    agarrar_cubo,
    avance_hasta_luz,
    motor_f,
)

reset_all()
golpear_pared(-30, 0.9, 0)
segundo_plano(reset_motores())
motor_f.run_angle(-600, 210)
avance_adelante(80, 100, 0)
giro(50, -45)
avance_adelante(50, 75, -45)
giro(50, 0)
avance_adelante(80, 900, 0)
giro(40, 90)
avance_reversa(30, 70, 90)
wait(2000)
#
## ----------------codigo colores-----------------#
hub.speaker.beep()
## ----------------codigo colores-----------------#
avance_adelante(50, 70, 90)
giro(50, 180)
avance_adelante(80, 640, 180)
motor_f.run_angle(600, 210)
giro(50, 90)
avance_reversa(80, 780, 90)
golpear_pared(-30, 0.9, 90)
# guiñada 0 otra vez


agarrar_cubo(1)
agarrar_cubo(2)
agarrar_cubo(3)
agarrar_cubo(4)
agarrar_cubo(5)
agarrar_cubo(6)
agarrar_cubo(7)
agarrar_cubo(8, ultimo_agarrado=True)


# ---------------codigo cubos-----------------

# soltar_cubo()
# bajar_barrera(180)
# wait(1000)
# subir_barrera(180)


motor_derecho.brake()
motor_izquierdo.brake()
