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

    # 1. Calcular los grados de motor necesarios para avanzar los mm deseados
    # Diámetro de la llanta = 62.5 mm
    grados_objetivo = (New_mm * 360) / (pi * diametro)
    # 2. El ciclo ahora evalúa los grados corregidos
    while (
        abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
    ) / 2 <= grados_objetivo:
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)
        motor_izquierdo.dc(NewSpeed - correction)
        motor_derecho.dc(NewSpeed + correction)
        wait(10)
        last_error = error


def avance_reversa(speed: int, mm: int, target_heading: int):
    NewSpeed = -abs(speed)
    New_mm = abs(mm)
    kp = 4
    kd = 10
    ki = 0.05
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)
    last_error = 0
    integral = 0

    # 1. Calcular los grados de motor necesarios para avanzar los mm deseados
    # Diámetro de la llanta = 62.5 mm
    grados_objetivo = (New_mm * 360) / (pi * diametro)
    # 2. El ciclo ahora evalúa los grados corregidos
    while (
        abs(motor_izquierdo.angle()) + abs(motor_derecho.angle())
    ) / 2 <= grados_objetivo:
        error = target_heading - hub.imu.heading()
        derivada = error - last_error
        integral += error

        correction = (kp * error + derivada * kd) + (integral * ki)
        motor_izquierdo.dc(NewSpeed - correction)
        motor_derecho.dc(NewSpeed + correction)
        wait(10)
        last_error = error


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

        if abs(error) < 0.1:  # Umbral de precisión
            break

        integral += error
        derivative = error - last_error
        correction = (kp * error) + (ki * integral) + (kd * derivative)

        correction = max(min(correction, speed), -speed)

        motor_izquierdo.dc(-correction)
        motor_derecho.dc(correction)

        last_error = error
