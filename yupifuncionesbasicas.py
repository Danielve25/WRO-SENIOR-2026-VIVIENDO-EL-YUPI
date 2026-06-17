from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi

hub = PrimeHub()
diametro = 62.5
motor_derecho = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_izquierdo = Motor(Port.E, Direction.CLOCKWISE)


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
    # Diámetro de la llanta = 62.5 mm
    grados_objetivo = (New_mm * 360) / (pi * diametro)

    # Parámetros de la rampa (15% al inicio y 15% al final)
    porcentaje_rampa = 0.15
    grados_rampa = grados_objetivo * porcentaje_rampa
    velocidad_minima = 20  # Evita que el robot se quede sin fuerza al arrancar o frenar

    # 2. Ciclo de control con rampas
    while True:
        # Calcular el progreso actual promedio
        grados_actuales = (
            abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
        ) / 2

        # Condición de salida del ciclo
        if grados_actuales >= grados_objetivo:
            break

        # --- CÁLCULO DE LA RAMPA DE VELOCIDAD ---
        if grados_actuales < grados_rampa:
            # Rampa de aceleración (Inicio)
            factor = grados_actuales / grados_rampa
            velocidad_limite = velocidad_minima + (NewSpeed - velocidad_minima) * factor
        elif grados_actuales > (grados_objetivo - grados_rampa):
            # Rampa de frenado (Fin)
            factor = (grados_objetivo - grados_actuales) / grados_rampa
            velocidad_limite = velocidad_minima + (NewSpeed - velocidad_minima) * factor
        else:
            # Velocidad de crucero (Centro)
            velocidad_limite = NewSpeed

        # --- CONTROL DE GIRO (PID) ---
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)

        # Aplicar la velocidad limitada por las rampas
        motor_izquierdo.dc(velocidad_limite - correction)
        motor_derecho.dc(velocidad_limite + correction)

        wait(10)
        last_error = error

    # Frenar los motores al salir del ciclo
    motor_izquierdo.stop()
    motor_derecho.stop()


def avance_reversa(speed: int, mm: int, target_heading: int):
    # Todo se calcula en valores negativos desde el inicio
    NewSpeed = -abs(speed)
    velocidad_minima = -20  # Límite mínimo negativo para no perder fuerza

    New_mm = abs(mm)
    kp = 4
    kd = 10
    ki = 0.05
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)
    last_error = 0
    integral = 0

    # 1. Calcular los grados de motor necesarios
    grados_objetivo = (New_mm * 360) / (pi * diametro)

    # Parámetros de la rampa al 15% (distancias siempre en positivo)
    porcentaje_rampa = 0.15
    grados_rampa = grados_objetivo * porcentaje_rampa

    # 2. Tu ciclo original
    while (
        abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
    ) / 2 <= grados_objetivo:
        grados_actuales = (
            abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
        ) / 2

        # --- CÁLCULO DE LA RAMPA (Todo en negativo) ---
        if grados_actuales < grados_rampa:
            # Rampa de aceleración: va desde -10 hasta NewSpeed (ej. -50)
            factor = grados_actuales / grados_rampa
            velocidad_limite = velocidad_minima + (NewSpeed - velocidad_minima) * factor
        elif grados_actuales > (grados_objetivo - grados_rampa):
            # Rampa de frenado: regresa desde NewSpeed hacia -10
            factor = (grados_objetivo - grados_actuales) / grados_rampa
            velocidad_limite = velocidad_minima + (NewSpeed - velocidad_minima) * factor
        else:
            # Velocidad de crucero negativa constante
            velocidad_limite = NewSpeed

        # --- TU PID ORIGINAL (Sin tocar una sola línea) ---
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)

        # --- TU ESTRUCTURA ORIGINAL DE MOTORES ---
        # Al ser 'velocidad_limite' negativa, mantiene tu lógica exacta de control
        motor_izquierdo.dc(velocidad_limite - correction)
        motor_derecho.dc(velocidad_limite + correction)

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
