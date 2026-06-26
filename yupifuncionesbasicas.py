from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi
from pybricks.tools import multitask, run_task

segundo_plano = run_task
hub = PrimeHub()
diametro = 62.5
motor_derecho = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_izquierdo = Motor(Port.E, Direction.CLOCKWISE)
motor_barrera = Motor(Port.F, gears=[12, 36, 28])
motor_garra = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_garra.control.stall_tolerances(speed=50, time=200)


def reset_imu():
    wait(100)  # Pequeña pausa para asegurar que el IMU esté listo
    hub.imu.reset_heading(0)
    wait(100)  # Otra pausa para estabilizar después del reset


def reset_all():
    reset_imu()
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)


async def reset_motores():
    # El 'await multitask' hace que ambas líneas arranquen simultáneamente
    # y pausa esta función hasta que ambos motores se hayan trabado.
    multitask(
        motor_garra.run_until_stalled(200, then=Stop.HOLD, duty_limit=60),
        motor_barrera.run_until_stalled(-200, then=Stop.HOLD, duty_limit=30),
    )


def subir_garra():
    motor_garra.run_until_stalled(-1110, then=Stop.HOLD)


def bajar_garra(fichas_Voladoras: bool = False):
    if fichas_Voladoras:
        motor_garra.run_angle(1110, 278)
    else:
        motor_garra.run_angle(600, 270)


def avance_adelante(speed: int, mm: int, target_heading: int):
    NewSpeed = abs(speed)
    New_mm = abs(mm)
    kp = 4
    kd = 10
    ki = 0.05
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)
    last_error = 0
    integral = 0

    # 1. Calcular los grados de motor necesarios
    # Diámetro de la llanta = 62.5 mm (Asegúrate de que 'pi' y 'diametro' estén definidos)
    grados_objetivo = (New_mm * 360) / (pi * diametro)

    # 2. Ciclo de control
    while True:
        # Calcular el progreso actual promedio
        grados_actuales = (
            abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
        ) / 2

        # Condición de salida del ciclo
        if grados_actuales >= grados_objetivo:
            break

        # --- CONTROL DE GIRO (PID) ---
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)

        # Aplicar la velocidad constante combinada con la corrección PID
        motor_izquierdo.dc(NewSpeed - correction)
        motor_derecho.dc(NewSpeed + correction)

        wait(10)
        last_error = error

    # Frenar los motores al salir del ciclo
    motor_izquierdo.stop()
    motor_derecho.stop()


def avance_reversa(speed: int, mm: int, target_heading: int):
    # Todo se calcula en valores negativos desde el inicio
    NewSpeed = -abs(speed)

    New_mm = abs(mm)
    kp = 4
    kd = 10
    ki = 0.05
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)
    last_error = 0
    integral = 0

    # 1. Calcular los grados de motor necesarios (Asegúrate de tener pi y diametro definidos)
    grados_objetivo = (New_mm * 360) / (pi * diametro)

    # 2. Ciclo de control
    while (
        abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
    ) / 2 <= grados_objetivo:

        # --- CONTROL DE GIRO (PID ORIGINAL) ---
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)

        # --- APLICACIÓN DE VELOCIDAD DIRECTA ---
        # Al usar 'NewSpeed' (que es negativa), el robot va en reversa a velocidad constante
        motor_izquierdo.dc(NewSpeed - correction)
        motor_derecho.dc(NewSpeed + correction)

        wait(10)
        last_error = error

    # Frenado final al salir del ciclo
    motor_izquierdo.stop()
    motor_derecho.stop()


def giro(speed, target_heading):
    kp = 4
    kd = 4
    ki = 0.01

    integral = 0
    last_error = 0

    while True:
        current_angle = hub.imu.heading()
        error = target_heading - current_angle

        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < 0.8:  # Umbral de precisión
            break

        integral += error
        derivative = error - last_error
        correction = (kp * error) + (ki * integral) + (kd * derivative)

        correction = max(min(correction, speed), -speed)

        motor_izquierdo.dc(-correction)
        motor_derecho.dc(correction)
        wait(10)
        last_error = error

    motor_izquierdo.stop()
    motor_derecho.stop()


def giro_un_motor(speed: int, target_heading: int, derecha: bool, izquierda: bool):
    kp = 4
    kd = 4
    ki = 0.01

    integral = 0
    last_error = 0

    while True:
        current_angle = hub.imu.heading()
        error = target_heading - current_angle

        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < 1:  # Umbral de precisión
            break

        integral += error
        derivative = error - last_error
        correction = (kp * error) + (ki * integral) + (kd * derivative)

        correction = max(min(correction, speed), -speed)

        # --- SELECCIÓN DE LLANTA ACTIVA ---
        if izquierda:
            # La llanta derecha se detiene por completo
            motor_derecho.stop()
            # La izquierda aplica la corrección completa (multiplicada por 2 para compensar la fuerza)
            motor_izquierdo.dc(-correction * 2)
        elif derecha:
            # La llanta izquierda se detiene por completo
            motor_izquierdo.stop()
            # La derecha aplica la corrección completa (multiplicada por 2 para compensar la fuerza)
            motor_derecho.dc(correction * 2)
        else:
            # Si escribes mal el nombre, por seguridad se detiene
            motor_izquierdo.stop()
            motor_derecho.stop()

        wait(10)
        last_error = error

    # Frenado total al terminar el giro
    motor_izquierdo.stop()
    motor_derecho.stop()
