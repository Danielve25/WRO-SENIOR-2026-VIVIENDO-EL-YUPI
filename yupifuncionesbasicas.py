from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi
from pybricks.tools import multitask, run_task, wait

segundo_plano = run_task
hub = PrimeHub()
diametro = 62
motor_derecho = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_izquierdo = Motor(Port.E, Direction.CLOCKWISE)
motor_f = Motor(Port.F)
motor_garra = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
motor_garra.control.stall_tolerances(speed=50, time=200)
color_sensor = ColorSensor(Port.B)


def soltar_cubo(grados: int = 60):
    # Relación original: [[20, 12, 36], [12, 20]]
    # Reducción total = (36 / 20) * (20 / 12) = 3.0
    # Para que la salida gire -600 grados a velocidad 20, el motor debe girar:
    grados_motor = -grados * 3  # -1800 grados
    velocidad_motor = 600  # 60

    motor_f.run_angle(velocidad_motor, grados_motor)
    wait(500)
    motor_f.run_angle(velocidad_motor, -grados_motor)
    motor_f.stop()  # Liberar el motor después de usarlo


def reset_imu():
    wait(100)  # Pequeña pausa para asegurar que el IMU esté listo
    hub.imu.reset_heading(0)
    wait(100)  # Otra pausa para estabilizar después del reset


async def reset_all():
    reset_imu()
    motor_derecho.reset_angle(0)
    motor_izquierdo.reset_angle(0)


def bajar_barrera(grados: int):
    # Relación real: 28 / 12 (aprox 2.333)
    grados_motor = -grados * (28 / 12)
    velocidad_motor = 600

    # Usamos run_angle para que respete los grados exactos que le pidas
    hub.speaker.beep()
    motor_f.run_angle(velocidad_motor, grados_motor)
    motor_f.stop()


def subir_barrera(grados: int):
    # Relación real: 28 / 12 (aprox 2.333)
    grados_motor = grados * (28 / 12)
    velocidad_motor = 600

    # Usamos run_angle para que respete los grados exactos que le pidas
    hub.speaker.beep()
    motor_f.run_angle(velocidad_motor, grados_motor)
    motor_f.stop()


async def reset_motores():
    # Hacemos exactamente lo mismo para la función asíncrona

    await multitask(
        motor_garra.run_until_stalled(200, then=Stop.HOLD, duty_limit=60),
        motor_f.run_until_stalled(600, then=Stop.HOLD, duty_limit=40),
    )
    motor_f.stop()


def subir_garra():
    motor_garra.run_until_stalled(-1110, then=Stop.HOLD)


def bajar_garra():
    motor_garra.run_angle(600, 270)


def seguir_linea_dc(speed: int, target_reflection: int, duration_ms: int):
    # Valores iniciales para calibrar con .dc()
    kp = 0.6
    kd = 4
    ki = 0.01

    integral = 0
    last_error = 0
    cronometro = StopWatch()

    # Encabezado para facilitar la creación de la gráfica
    print("tiempo_ms, luz_actual, luz_objetivo, error")

    while cronometro.time() < duration_ms:
        current_reflection = color_sensor.reflection()
        error = target_reflection - current_reflection

        integral += error
        derivative = error - last_error
        correction = (kp * error) + (ki * integral) + (kd * derivative)

        # Calculamos la potencia de cada motor
        power_izq = speed + correction
        power_der = speed - correction

        # ¡CRÍTICO PARA .DC()!
        # Limitamos los valores entre -100 y 100 para evitar errores de saturación
        power_izq = max(-100, min(100, power_izq))
        power_der = max(-100, min(100, power_der))

        # Aplicamos el ciclo de trabajo directo
        motor_izquierdo.dc(power_izq)
        motor_derecho.dc(power_der)

        # Imprime los datos: Tiempo, luz leída por el sensor, el objetivo y el error
        print(
            cronometro.time(),
            ",",
            current_reflection,
            ",",
            target_reflection,
            ",",
            error,
        )

        wait(10)
        last_error = error

    # Frenado al final del tramo
    motor_izquierdo.dc(0)
    motor_derecho.dc(0)


# 2. Definición de la Función PID
def seguir_linea_dinamico(speed: int, target_reflection: int, duration_ms: int):
    # Valores base
    kp_base = 0.4  # Un valor suave para cuando va bien alineado
    kd = 4
    ki = 0  # ¡Apagado para alta velocidad!

    # Factor de agresividad (qué tan rápido sube la rampa)
    rampa_kp = 0.02

    integral = 0
    last_error = 0
    cronometro = StopWatch()

    while cronometro.time() < duration_ms:
        current_reflection = color_sensor.reflection()
        error = target_reflection - current_reflection

        # --- AQUÍ ESTÁ TU RAMPA DE CORRECCIÓN ---
        # abs(error) convierte cualquier error (positivo o negativo) en un número positivo.
        # Si el error es 0, kp_dinamico = 0.4
        # Si el error es 30 (muy lejos), kp_dinamico = 0.4 + (30 * 0.02) = 1.0
        kp_dinamico = kp_base + (abs(error) * rampa_kp)

        integral += error
        derivative = error - last_error

        # Usamos el kp_dinamico en lugar de un kp fijo
        correction = (kp_dinamico * error) + (ki * integral) + (kd * derivative)

        # Calculamos y limitamos la potencia de cada motor
        power_izq = max(-100, min(100, speed + correction))
        power_der = max(-100, min(100, speed - correction))

        motor_izquierdo.dc(power_izq)
        motor_derecho.dc(power_der)

        wait(10)
        last_error = error

    motor_izquierdo.dc(0)
    motor_derecho.dc(0)


# 3. Ejemplo de uso en el programa principal
# El robot seguirá la línea a 200°/s durante 5 segundos (5000 ms)


# Aquí puedes añadir más acciones después de seguir la línea


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
