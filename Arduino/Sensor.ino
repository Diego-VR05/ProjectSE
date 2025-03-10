#include <Wire.h>
#include <VL53L0X.h>

// Definición de la clase SensorVL53L0X
class SensorVL53L0X {
    private:
        VL53L0X sensor;
        TwoWire *i2c;
        
    public:
        // Constructor para inicializar el I2C en ESP32
        SensorVL53L0X(TwoWire *wire, uint8_t sda, uint8_t scl) {
            i2c = wire;
            i2c->begin(sda, scl);
        }
        
        // Método para iniciar el sensor
        void iniciar() {
            sensor.setTimeout(500);
            if (!sensor.init()) {
                Serial.println("No se pudo inicializar el VL53L0X");
                while (true); // Detener ejecución si falla
            }
            sensor.startContinuous();
        }
        
        // Método para obtener la distancia medida
        int leerDistancia() {
            return sensor.readRangeContinuousMillimeters();
        }
        
        // Método para verificar si hay error en la lectura
        bool hayError() {
            return sensor.timeoutOccurred();
        }
};

// Instanciación del sensor usando I2C en ESP32
SensorVL53L0X sensorDistancia(&Wire, 21, 22); // SDA = 21, SCL = 22

void setup() {
    Serial.begin(115200);
    iniciarSensor();
}

// Función para iniciar el sensor
void iniciarSensor() {
    sensorDistancia.iniciar();
}

// Función para leer y mostrar la distancia
void mostrarDistancia() {
    int distancia = sensorDistancia.leerDistancia();
    
    if (sensorDistancia.hayError()) {
        Serial.println("Error de tiempo de espera en la lectura");
    } else {
        Serial.print("Distancia: ");
        Serial.print(distancia);
        Serial.println(" mm");
    }
}

void loop() {
    mostrarDistancia();
    delay(500);
}
