import time
from machine import Pin, I2C, PWM
# import vl53l0x  # Descomenta si tienes la librería VL53L0X para MicroPython

# Definición de la clase del sensor VL53L0X
class SensorVL53L0X:
    def __init__(self, sda=21, scl=22):
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)
        # self.sensor = vl53l0x.VL53L0X(self.i2c)  # Inicializa el sensor si tienes la librería
        pass

    def leer_distancia(self):
        # return self.sensor.read() / 10.0  # Leer distancia en cm si la librería está disponible
        return 20.0  # Valor de prueba si no hay sensor

# Definición de la clase Motor
class Motor:
    def __init__(self, in1, in2, en, freq=1000, velocidad=128):
        self.IN1 = Pin(in1, Pin.OUT)
        self.IN2 = Pin(in2, Pin.OUT)
        self.EN = PWM(Pin(en), freq=freq)
        self.EN.duty(0)  # Inicialmente apagado

    def set_velocidad(self, velocidad, adelante=True):
        self.IN1.value(1 if adelante else 0)
        self.IN2.value(0 if adelante else 1)
        self.EN.duty(velocidad)

    def detener(self):
        self.EN.duty(0)

# Inicialización del sensor y motores
sensor_distancia = SensorVL53L0X()
motorA = Motor(18, 19, 5)
motorB = Motor(21, 22, 23)

# **FASE INICIAL**: Giro de 180° antes de cualquier acción
print("Realizando giro inicial de 180°...")
motorA.set_velocidad(128, False)  # Motor A retrocede
motorB.set_velocidad(128, True)   # Motor B avanza
time.sleep(2)  # Ajusta el tiempo según la velocidad y diseño de las ruedas
motorA.detener()
motorB.detener()
time.sleep(1)  # Pausa breve antes de continuar

while True:
    distancia = sensor_distancia.leer_distancia()
    print(f"Distancia: {distancia:.2f} cm")

    if distancia <= 15:
        print("Obstáculo detectado, girando a la izquierda...")
        motorA.set_velocidad(128, False)  # Motor A retrocede
        motorB.set_velocidad(128, True)   # Motor B avanza
        time.sleep(1)  # Ajusta el tiempo de giro si es necesario
    else:
        print("Vía libre, avanzando...")
        motorA.set_velocidad(128, True)  # Ambos motores avanzan
        motorB.set_velocidad(128, True)
    
    time.sleep(0.2)  # Pequeña pausa para evitar lecturas demasiado rápidas
