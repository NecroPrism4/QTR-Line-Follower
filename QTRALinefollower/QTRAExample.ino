#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


const int PWMA = 5; // Pin para la señal PWM del motor A
const int AIN2 = 4; // Pin para la entrada 2 del motor A
const int AIN1 = 9; // Pin para la entrada 1 del motor A


const int BIN1 = 7; // Pin para la entrada 1 del motor B
const int BIN2 = 8; // Pin para la entrada 2 del motor B
const int PWMB = 6; // Pin para la señal PWM del motor B

// Definir las constantes del controlador PID
float kp = 0.1;  // Ganancia proporcional
//float ki = 0.01; // Ganancia integral
float kd = 0.05; // Ganancia derivativa

// Variables para el controlador PID
float previousError = 0;
float integral = 0;

#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line

void setup()
{
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);

  CalibrateQTR();
  ConfigureMotors();
}

int lastError = 0;

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  //PrintQTRValues(position);

  int error = position - 3600;

  int motorSpeed = kp * error + kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;

  // Aplicar la salida del controlador a los motores
  controlMotorA(constrain(rightMotorSpeed, 0, rightMaxSpeed));
  controlMotorB(constrain(leftMotorSpeed, 0, leftMaxSpeed));
}

  void controlMotorA(int speed){
    Serial.print("Right: ");
    Serial.print(speed/3); // Imprime la velocidad

    // Gira el motor B en la dirección opuesta durante 2 segundos
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed/3); // Velocidad máxima
  }

  void controlMotorB(int speed){
    Serial.print("    -     ");
    Serial.print("Left: ");
    Serial.println(speed/3);
    
    // Gira el motor B en la dirección opuesta durante 2 segundos
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed/3); // Velocidad máxima
  }

  void CalibrateQTR(){ 

    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(2);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++)
    {
      qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
  }

  void ConfigureMotors(){
    // Configura los pines como salidas
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
  }

  void PrintQTRValues(uint16_t position){
    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);

    delay(250);
  }
