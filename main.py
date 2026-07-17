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
    giro_arco,
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
motor_f.run_angle(-600, 240)
hub.speaker.beep()
avance_adelante(80, 100, 0)
giro(50, -45)
avance_adelante(50, 75, -45)
giro(50, 0)
avance_adelante(80, 900, 0)
subir_barrera(30)
giro(40, 90)

avance_reversa(30, 70, 90)
wait(2000)
#
## ----------------codigo colores-----------------#
hub.speaker.beep()
## ----------------codigo colores-----------------#
avance_adelante(50, 75, 90)

motor_f.run_time(speed=-1000, time=1200)
avance_reversa(20, 18, 90)
giro(30, 0)
avance_adelante(60, 380, 0)
giro_un_motor(30, -90, 0, 1)
subir_barrera(90)
giro_un_motor(30, 0, 0, 1)
avance_adelante(60, 160, 0)
giro_un_motor(30, -90, 0, 1)
golpear_pared(-60, 1, -90)
avance_adelante(70, 590, 0)
giro(50, 90)
motor_f.run_time(speed=-1000, time=1500)
avance_adelante(60, 100, 90)
subir_barrera(90)
avance_adelante(40, 50, 90)
motor_f.run_time(speed=-1000, time=1500)
giro(50, -90)
avance_adelante(60, 475, -90)
subir_barrera(90)
avance_reversa(70, 230, -90)
giro(50, 0)
avance_adelante(70, 240, 0)


giro(50, 90)
motor_f.run_time(speed=-1000, time=1300)
avance_adelante(50, 200, 90)
subir_barrera(70)
avance_adelante(40, 30, 90)
motor_f.run_time(speed=-1000, time=1000)
avance_reversa(70, 150, 90)
giro(50, 180)
golpear_pared(-90, 0.5, 180)
avance_adelante(80, 720, 0)

giro_arco(50, 260, 90)
avance_adelante(90, 1000, 90)


exit()

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
agarrar_cubo(8)
avance_adelante(60, 310, 0)
giro_un_motor(30, -90, 0, 1)


# ---------------codigo cubos-----------------

# soltar_cubo()
# bajar_barrera(180)
# wait(1000)
# subir_barrera(180)


motor_derecho.brake()
motor_izquierdo.brake()
