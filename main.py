from yupifuncionesbasicas import avance_adelante, giro, avance_reversa, hub, wait

hub.imu.reset_heading(0)


avance_adelante(80, 300, 0)
giro(50, 90)
avance_adelante(80, 300, 90)
giro(50, 180)
avance_adelante(80, 300, 180)
giro(50, 270)
avance_adelante(80, 300, 270)
giro(50, 360)

print(
    hub.imu.heading()
)  # Debería imprimir aproximadamente 180 grados, confirmando el giro correctos
