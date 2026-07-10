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
)

reset_all()
golpear_pared(-30, 0.7, 0)
segundo_plano(reset_motores())
avance_adelante(80, 1050, 0)
giro(50, 90)
avance_reversa(50, 130, 90)
wait(2000)

# ----------------codigo colores-----------------#

# ----------------codigo colores-----------------#

avance_adelante(50, 100, 90)
giro(50, 180)
avance_adelante(80, 640, 180)
giro(50, 90)
avance_reversa(80, 780, 90)
golpear_pared(-30, 0.7, 90)
# guiñada 0 otra vez
avance_adelante(30, 230, 0)
giro(30, 90)

subir_garra(primeravez=True)
avance_adelante(30, 115, 90)
subir_garra(agarreMovida=True)
avance_reversa(30, 32, 90)
subir_garra()
wait(50)
bajar_garra()
giro(50, 180)
avance_reversa(40, 50, 180)

giro(30, 90)

avance_adelante(30, 40, 90)
subir_garra(agarreMovida=True)
avance_reversa(30, 35, 90)
subir_garra()
wait(50)
bajar_garra()
giro(50, 180)
avance_reversa(40, 205, 180)
soltar_cubo()
avance_reversa(40, 30, 180)
soltar_cubo()


# ---------------codigo cubos-----------------

# soltar_cubo()
# bajar_barrera(180)
# wait(1000)
# subir_barrera(180)


motor_derecho.brake()
motor_izquierdo.brake()
