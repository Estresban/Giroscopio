#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>

MPU6050 mpu;
Servo servoPitch; // Servo eje X (adelante/atrás)
Servo servoRoll;  // Servo eje Y (izquierda/derecha)

// Variables de control del DMP
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

// Variables de orientación
Quaternion q;
VectorFloat gravity;
float ypr[3]; // [yaw, pitch, roll]

// Factor de ganancia (ajusta según la longitud de los brazos de tus servos)
float Kp = 1.0; 

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // I2C a 400kHz

  Serial.begin(115200);
  
  servoPitch.attach(9);
  servoRoll.attach(10);
  
  // Posición inicial nivelada (90 grados)
  servoPitch.write(90);
  servoRoll.write(90);

  mpu.initialize();
  pinMode(2, INPUT);

  devStatus = mpu.dmpInitialize();

  // Offsets básicos (puedes calibrar los tuyos propios para mayor precisión)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("Error inicializando DMP: ");
    Serial.println(devStatus);
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convertir radianes a grados
    float pitch = ypr[1] * 180 / M_PI;
    float roll  = ypr[2] * 180 / M_PI;

    // Calcular ángulo de compensación (invierte el signo +/- si el servo gira al revés)
    int anguloPitch = 90 - (pitch * Kp);
    int anguloRoll  = 90 - (roll * Kp);

    // Limitar recorrido para no forzar la mecánica de los servos
    anguloPitch = constrain(anguloPitch, 30, 150);
    anguloRoll  = constrain(anguloRoll, 30, 150);

    servoPitch.write(anguloPitch);
    servoRoll.write(anguloRoll);
  }
}
