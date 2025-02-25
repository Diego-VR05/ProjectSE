class Motor {
public:
    Motor(int in1, int in2, int en, int channel) : IN1(in1), IN2(in2), EN(en), channel(channel) {
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(EN, OUTPUT);
        ledcSetup(channel, 100, 8); // Canal PWM a 1kHz, 8-bit
        ledcAttachPin(EN, channel);
    }

    void setSpeed(int speed, bool forward) {
        digitalWrite(IN1, forward ? HIGH : LOW);
        digitalWrite(IN2, forward ? LOW : HIGH);
        ledcWrite(channel, speed);
    }

private:
    int IN1, IN2, EN, channel;
};

Motor motorA(18, 19, 5, 0);  // GPIO 18, 19, 5 para ESP32
Motor motorB(21, 22, 23, 1); // GPIO 21, 22, 23 para ESP32

void setup() {
    // Inicialización automática en constructor de Motor
}

void loop() {
    // Mover motores adelante a velocidad media durante 1 segundo
    motorA.setSpeed(128, true);
    motorB.setSpeed(128, true);
    delay(1000);
    
    // Mover motores atrás a velocidad media durante 1 segundo
    motorA.setSpeed(128, false);
    motorB.setSpeed(128, false);
    delay(1000);
}
