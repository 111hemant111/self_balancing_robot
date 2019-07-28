
                    //         EE503 Course Project           //
                    //    CODE DEVELOPED BY HEMANT KURUVA     //
                    //               20077636                 //
                    //                                        //


#include "Wire.h"
#include "math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <NewPing.h>
#include <LiquidCrystal.h>

const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define leftMotorB   6
#define leftMotorA   7
#define rightMotorB  5
#define rightMotorA  4

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp  60
#define Kd  0.1
#define Ki  10
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t acceleroY, acceleroZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorB, leftMotorSpeed);
    digitalWrite(leftMotorA, LOW);
  }
  else {
    /////Direction Reversal
    analogWrite(leftMotorB, 255 + leftMotorSpeed);
    digitalWrite(leftMotorA, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorB, rightMotorSpeed);
    digitalWrite(rightMotorA, LOW);
  }
  else {
    /////Direction Reversal
    analogWrite(rightMotorB, 255 + rightMotorSpeed);
    digitalWrite(rightMotorA, HIGH);
  }
}



/////INITIALIZING PID CONTROLLER/////

void init_PID() {  
  // initialize Timer1
  cli();          // Disabling the global interrupts
  TCCR1A = 0;     // Setting the Timer/Counter Control Register A to 0
  TCCR1B = 0;     // Setting the Timer/Counter Control Register B to 0
  // Setting Compare Match Register to set sample time 5ms
  OCR1A = 9999;    
  // Turn on CTC mode (Clear Timer on Compare mode)
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // Enabling global interrupts
}

void setup() {

  /////INITIALIZING THE LCD DISPLAY/////
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("     Hello!     ");
  lcd.setCursor(0,1);
  lcd.print("   Dr. Sydney   ");
  
  // SETTING THE MOTOR PINS TO OUTPUT MODE/////
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  
  
  pinMode(13, OUTPUT); // Status LED as Output



  /////INITIALIZING THE MPU6050 AND SETTING THE OFFSET VALUES
  /////Found out these offset values from trial and error
  
  mpu.initialize();
  mpu.setYAccelOffset(-630);
  mpu.setZAccelOffset(518);
  mpu.setXGyroOffset(23);
  init_PID(); // Initializing the PID sampling loop
}

void loop() {
  // Reading the acceleration and gyroscope values
  acceleroY = mpu.getAccelerationY();
  acceleroZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  motorPower = constrain(motorPower, -255, 255); // Setting the motor power after constraining it
  setMotors(motorPower, motorPower);
  // Measure distance every 100 milliseconds
  if((count%20) == 0){
    distanceCm = sonar.ping_cm();
  }
  if((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower); //Changing the direction of the robot due to the obstacle
  }
}


// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // Calculate the angle of inclination
  accAngle = atan2(acceleroY, acceleroZ)*RAD_TO_DEG; //From the mathematical equation
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle); //Estimation of the current angle
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //Calculating the PID Controller output by using the Kp, Ki, Kd values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  count++;
  
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13)); // Toggling the LED on pin13 every second
  }
}
