from machine import Pin, PWM, I2C
import time
import vl53l0x
import math

# --- Configuración de pines ---
# Motores
ENA = PWM(Pin(5), freq=1000)       # PWM Motor A
IN1 = Pin(18, Pin.OUT)             # Dirección Motor A
IN2 = Pin(19, Pin.OUT)             # Dirección Motor A
ENB = PWM(Pin(25), freq=1000)      # PWM Motor B
IN3 = Pin(4, Pin.OUT)              # Dirección Motor B
IN4 = Pin(23, Pin.OUT)             # Dirección Motor B

# Sensores ToF
XSHUT_FRONTAL = Pin(32, Pin.OUT)   # Control XSHUT sensor frontal
XSHUT_LATERAL = Pin(33, Pin.OUT)   # Control XSHUT sensor lateral

# --- Configuración I2C (compartido para ToF y MPU6050) ---
# MPU6050 usa los mismos pines I2C que los sensores ToF:
# - SCL: GPIO 22
# - SDA: GPIO 21
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# --- Clase para el MPU6050 (Giroscopio/Acelerómetro) ---
class MPU6050:
    def _init_(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        # Inicializar el MPU6050
        self.i2c.writeto_mem(self.address, 0x6B, b'\x00')  # PWR_MGMT_1
        self.i2c.writeto_mem(self.address, 0x1B, b'\x08')  # GYRO_CONFIG (500°/s)
        self.i2c.writeto_mem(self.address, 0x1C, b'\x08')  # ACCEL_CONFIG (4g)
        
        # Calibración
        self.gyro_offset = [0, 0, 0]
        self.calibrate(100)
    
    def calibrate(self, samples=100):
        print("Calibrando giroscopio... No muevas el robot.")
        x, y, z = 0, 0, 0
        for _ in range(samples):
            gx, gy, gz = self.read_gyro()
            x += gx
            y += gy
            z += gz
            time.sleep_ms(10)
        self.gyro_offset = [x/samples, y/samples, z/samples]
        print(f"Offsets calculados: X={self.gyro_offset[0]:.2f}, Y={self.gyro_offset[1]:.2f}, Z={self.gyro_offset[2]:.2f}")
    
    def read_gyro(self):
        data = self.i2c.readfrom_mem(self.address, 0x43, 6)
        gx = (data[0] << 8 | data[1]) / 65.5 - self.gyro_offset[0]
        gy = (data[2] << 8 | data[3]) / 65.5 - self.gyro_offset[1]
        gz = (data[4] << 8 | data[5]) / 65.5 - self.gyro_offset[2]
        return gx, gy, gz
    
    def get_rotation(self, dt):
        gx, gy, gz = self.read_gyro()
        return gz * dt  # Retorna el ángulo en grados

# --- Inicialización de sensores VL53L0X ---
def init_sensors():
    # Apagar ambos sensores primero
    XSHUT_FRONTAL.value(0)
    XSHUT_LATERAL.value(0)
    time.sleep_ms(100)
    
    # Encender y configurar sensor frontal
    XSHUT_FRONTAL.value(1)
    time.sleep_ms(100)
    sensor_frontal = vl53l0x.VL53L0X(i2c)
    sensor_frontal.start()
    
    # Encender y configurar sensor lateral con nueva dirección
    XSHUT_LATERAL.value(1)
    time.sleep_ms(100)
    sensor_lateral = vl53l0x.VL53L0X(i2c)
    sensor_lateral.set_address(0x30)
    sensor_lateral.start()
    
    return sensor_frontal, sensor_lateral

# --- Funciones de movimiento ---
def mover_adelante(velocidad=800):
    ENA.duty(velocidad)
    IN1.value(1)
    IN2.value(0)
    ENB.duty(velocidad)
    IN3.value(1)
    IN4.value(0)

def detener_motores():
    ENA.duty(0)
    IN1.value(0)
    IN2.value(0)
    ENB.duty(0)
    IN3.value(0)
    IN4.value(0)

def girar_90(direccion, mpu):
    velocidad = 700
    ENA.duty(velocidad)
    ENB.duty(velocidad)
    
    if direccion == 1:  # Derecha
        IN1.value(1)
        IN2.value(0)
        IN3.value(0)
        IN4.value(1)
        print("Iniciando giro de 90° a la derecha")
    else:  # Izquierda
        IN1.value(0)
        IN2.value(1)
        IN3.value(1)
        IN4.value(0)
        print("Iniciando giro de 90° a la izquierda")
    
    # Control de giro con giroscopio
    angulo_actual = 0
    angulo_objetivo = 90  # Grados
    tiempo_anterior = time.ticks_ms()
    tolerancia = 5  # Margen de error aceptable
    
    while abs(angulo_actual) < angulo_objetivo - tolerancia:
        tiempo_actual = time.ticks_ms()
        dt = (tiempo_actual - tiempo_anterior) / 1000  # Convertir a segundos
        tiempo_anterior = tiempo_actual
        
        rotacion = mpu.get_rotation(dt)
        angulo_actual += abs(rotacion)
        print(f"Ángulo girado: {angulo_actual:.1f}°", end='\r')
        time.sleep_ms(10)
    
    detener_motores()
    print(f"\nGiro completado. Ángulo final: {angulo_actual:.1f}°")

# --- Programa principal ---
def main():
    print("Iniciando robot ESP32...")
    
    try:
        # Inicializar sensores
        sensor_frontal, sensor_lateral = init_sensors()
        print("Sensores ToF inicializados correctamente.")
        
        mpu = MPU6050(i2c)
        print("MPU6050 inicializado y calibrado.")
        
        UMBRAL = 150  # Distancia en mm para detección de obstáculos
        
        while True:
            df = sensor_frontal.read()
            dl = sensor_lateral.read()
            
            print(f"Frontal: {df:4} mm | Lateral: {dl:4} mm", end='\r')
            
            if df > UMBRAL:
                mover_adelante()
            else:
                detener_motores()
                time.sleep_ms(500)  # Pequeña pausa antes de girar
                
                # Decidir dirección de giro basado en sensor lateral
                if dl > UMBRAL:
                    girar_90(1, mpu)  # Derecha
                else:
                    girar_90(-1, mpu)  # Izquierda
                
                time.sleep_ms(500)  # Pausa después del giro
            
            time.sleep_ms(100)
            
    except Exception as e:
        print("\nError:", e)
        detener_motors()
    finally:
        detener_motores()
        print("Programa terminado")

# --- Ejecutar solo si es archivo principal ---
if _name_ == "_main_":
    main()