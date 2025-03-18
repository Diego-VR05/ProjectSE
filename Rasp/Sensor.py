import time
from machine import Pin, I2C
import vl53l0x  # Si tienes la librería VL53L0X para MicroPython, la puedes descomentar

class SensorVL53L0X:
    def __init__(self, sda=21, scl=22):
        # Configurar I2C en ESP32 con los pines adecuados
        self.i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)
        self.sensor = vl53l0x.VL53L0X(self.i2c)  # Si tienes la librería, inicializa el sensor
        pass

    def leer_distancia(self):
        return self.sensor.range  # Leer distancia en metros (si la librería está disponible)
        #return 0.0  # Devuelve un valor de prueba si no hay sensor

if __name__ == "__main__":
    # Instanciar el sensor con los pines SDA = 21 y SCL = 22 en ESP32
    sensor_distancia = SensorVL53L0X()

    while True:
        distancia = (sensor_distancia.leer_distancia())/10
        print(f"Distancia: {distancia:.1f} cm")
        time.sleep(0.5)

