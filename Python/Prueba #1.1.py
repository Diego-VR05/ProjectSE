from machine import Pin, PWM, I2C
import time
import vl53l0x  # Asegúrate de tener este archivo en tu ESP32

# --- Configuración de pines ---
ENA = PWM(Pin(5), freq=1000)       # PWM Motor A
IN1 = Pin(18, Pin.OUT)             # Dirección Motor A
IN2 = Pin(19, Pin.OUT)             # Dirección Motor A
ENB = PWM(Pin(25), freq=1000)      # PWM Motor B
IN3 = Pin(4, Pin.OUT)              # Dirección Motor B
IN4 = Pin(23, Pin.OUT)             # Dirección Motor B

# Pines de control para sensores ToF
XSHUT_FRONTAL = Pin(32, Pin.OUT)  # Control XSHUT sensor frontal
XSHUT_LATERAL = Pin(33, Pin.OUT)  # Control XSHUT sensor lateral

# --- Configuración I2C ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# --- Inicialización de sensores VL53L0X ---
def init_sensors():
    XSHUT_FRONTAL.value(0)
    XSHUT_LATERAL.value(0)
    time.sleep_ms(100)
    
    XSHUT_FRONTAL.value(1)
    time.sleep_ms(100)
    sensor_frontal = vl53l0x.VL53L0X(i2c)
    sensor_frontal.start()
    
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

def girar_90(direccion, tiempo_giro=1000):
    velocidad = 700
    ENA.duty(velocidad)
    ENB.duty(velocidad)
    
    if direccion == 1:  # Derecha
        IN1.value(1)
        IN2.value(0)
        IN3.value(0)
        IN4.value(1)
    else:  # Izquierda
        IN1.value(0)
        IN2.value(1)
        IN3.value(1)
        IN4.value(0)
    
    time.sleep_ms(tiempo_giro)
    detener_motores()

# --- Programa principal ---
def main():
    print("Iniciando robot ESP32...")
    
    try:
        sensor_frontal, sensor_lateral = init_sensors()
        print("Sensores VL53L0X inicializados correctamente.")
        
        UMBRAL = 150  # Distancia en mm
        
        while True:
            df = sensor_frontal.read()
            dl = sensor_lateral.read()
            
            print("Frontal:", df, "mm | Lateral:", dl, "mm")
            
            if df > UMBRAL:
                mover_adelante()
            else:
                detener_motores()
                time.sleep_ms(500)
                
                if dl > UMBRAL:
                    print("Girando 90° a la derecha")
                    girar_90(1)
                else:
                    print("Girando 90° a la izquierda")
                    girar_90(-1)
            
            time.sleep_ms(100)
            
    except Exception as e:
        print("Error:", e)
        detener_motores()
    finally:
        detener_motores()
        print("Programa terminado")

# --- Ejecutar solo si es archivo principal ---
main()

