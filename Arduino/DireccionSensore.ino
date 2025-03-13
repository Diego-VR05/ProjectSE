#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// Pines XSHUT de cada sensor
#define XSHUT1  32  
#define XSHUT2  33  
#define XSHUT3  25  

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Inicializa el I2C en ESP32

    // Configurar pines XSHUT como salida
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    // Apagar todos los sensores
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
    delay(10);

    // Inicializar sensor 1
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    sensor1.init();
    sensor1.setAddress(0x30);  // Nueva dirección para sensor 1

    // Inicializar sensor 2
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    sensor2.init();
    sensor2.setAddress(0x31);  // Nueva dirección para sensor 2

    // Inicializar sensor 3
    digitalWrite(XSHUT3, HIGH);
    delay(10);
    sensor3.init();
    sensor3.setAddress(0x32);  // Nueva dirección para sensor 3

    // Iniciar los sensores en modo continuo
    sensor1.startContinuous();
    sensor2.startContinuous();
    sensor3.startContinuous();
}

void loop() {
    Serial.print("Sensor 1: ");
    Serial.print(sensor1.readRangeContinuousMillimeters());
    Serial.print(" mm, Sensor 2: ");
    Serial.print(sensor2.readRangeContinuousMillimeters());
    Serial.print(" mm, Sensor 3: ");
    Serial.print(sensor3.readRangeContinuousMillimeters());
    Serial.println(" mm");

    delay(500);
}
// llgue yo y voy a empeza a acomodar el codgio
//ksjdfwb
//wkjeb
