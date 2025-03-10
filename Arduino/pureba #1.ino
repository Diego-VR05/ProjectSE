#include <Wire.h>
#include <VL53L0X.h>

// Clase para manejar el sensor VL53L0X
class SensorVL53L0X {
  private:
    VL53L0X sensor;

  public:
    void iniciar() {
        Wire.begin();
        sensor.init();
        sensor.setTimeout(500);
        sensor.startContinuous();
    }

    float leerDistancia() {
        return sensor.readRangeContinuousMillimeters() / 10.0; // Convertir a cm
    }
};

// Clase para controlar los motores
class Motor {
  private:
    int IN1, IN2, EN;

  public:
    Motor(int in1, int in2, int en) {
        IN1 = in1;
        IN2 = in2;
        EN = en;
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(EN, OUTPUT);
    }

    void mover(int velocidad, bool adelante) {
        digitalWrite(IN1, adelante ? HIGH : LOW);
        digitalWrite(IN2, adelante ? LOW : HIGH);
        analogWrite(EN, velocidad);
    }

    void detener() {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(EN, 0);
    }
};

// Definir objetos
SensorVL53L0X sensorDistancia;
Motor motorA(18, 19, 5);
Motor motorB(21, 22, 23);

void setup() {
    Serial.begin(115200);
    sensorDistancia.iniciar();

    // Giro inicial de 180°
    motorA.mover(128, true);  // Motor A hacia adelante
    motorB.mover(128, false); // Motor B hacia atrás
    delay(1000); // Ajustar el tiempo según la velocidad

    motorA.detener();
    motorB.detener();
    delay(500);
}

void loop() {
    float distancia = sensorDistancia.leerDistancia();
    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.println(" cm");

    if (distancia <= 15) {
        // Si hay obstáculo, girar a la izquierda
        motorA.mover(128, false);
        motorB.mover(128, true);
    } else {
        // Si no hay obstáculo, avanzar
        motorA.mover(128, true);
        motorB.mover(128, true);
    }

    delay(100);
}
