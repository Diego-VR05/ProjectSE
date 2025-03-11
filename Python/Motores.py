
from machine import Pin, PWM
import time

class Motor:
    def __init__(self, in1, in2, en, freq=1000, velocidad=128):
        self.IN1 = Pin(in1, Pin.OUT)
        self.IN2 = Pin(in2, Pin.OUT)
        self.EN = PWM(Pin(en), freq=freq)  # Configura PWM sin duty
        self.EN.duty(velocidad)  # Asigna el ciclo de trabajo inicial

    def set_velocidad(self, velocidad, fuerza):
        self.IN1.value(1 if fuerza else 0)
        self.IN2.value(0 if fuerza else 1)
        self.EN.duty(velocidad)

# Definición de motores con los pines correspondientes
motorA = Motor(18, 19, 5)
motorB = Motor(21, 22, 23)

while True:
    # Mover motores adelante a velocidad media durante 1 segundo
    motorA.set_velocidad(128, True)
    motorB.set_velocidad(128, True)
    time.sleep(1)
    
    # Mover motores atrás a velocidad media durante 1 segundo
    motorA.set_velocidad(128, False)
    motorB.set_velocidad(128, False)
    time.sleep(1)
