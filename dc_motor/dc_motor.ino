#include <Wire.h> // Si estás utilizando una placa Arduino, no es necesario incluir Wire.h

const int PWMA = 5; // Pin para la señal PWM del motor A
const int AIN2 = 4; // Pin para la entrada 2 del motor A
const int AIN1 = 9; // Pin para la entrada 1 del motor A


const int BIN1 = 7; // Pin para la entrada 1 del motor B
const int BIN2 = 8; // Pin para la entrada 2 del motor B
const int PWMB = 6; // Pin para la señal PWM del motor B

void setup() {
  // Configura los pines como salidas
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void loop() {
  // Gira el motor B en la dirección opuesta durante 2 segundos
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 50); // Velocidad máxima
 
  // Gira el motor B en la dirección opuesta durante 2 segundos
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 50); // Velocidad máxima
}