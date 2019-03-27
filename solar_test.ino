/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#define DISTANCE false

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SimpleKalmanFilter.h>
#define TRIG_PIN 12
#define ECHO_PIN 11

#if DISTANCE 
#include "SR04.h"
#endif


int dist;

#if DISTANCE
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
#endif


#include <Adafruit_SleepyDog.h>

int LDR1 = A0;
int LDR2 = A1;
int LDR3 = A2;
int LDR4 = A3;
int motor_speed = 255;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 10;
long refresh_time;

// Ease start vars
enum state {
  FWD,
  LEFT,
  RIGHT,
  OFF
};

state current_direction = OFF;
state prev_direction = OFF;
int spd = 0;
int max_spd = 175;

SimpleKalmanFilter simpleKalmanFilter1(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter2(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter3(2, 2, 0.01);
SimpleKalmanFilter simpleKalmanFilter4(2, 2, 0.01);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  //sleepydog setup
  Serial.begin(115200);
  while(!Serial); // wait for Arduino Serial Monitor (native USB boards)

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake

  //motor setup
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  myMotor2->setSpeed(150);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);


}

void loop() {
    // read a reference value from A0 and map it from 0 to 100
  float real_value1 = analogRead(A0)/1024.0 * 100.0;
  
  // calculate the estimated value with Kalman Filter
  float estimated_value1 = simpleKalmanFilter1.updateEstimate(real_value1);

  float real_value2 = analogRead(A1)/1024.0 * 100.0;
  
  float estimated_value2 = simpleKalmanFilter2.updateEstimate(real_value2);
  
  float real_value3 = analogRead(A2)/1024.0 * 100.0;
  
  float estimated_value3 = simpleKalmanFilter3.updateEstimate(real_value3);
  
  float real_value4 = analogRead(A3)/1024.0 * 100.0;
  
  float estimated_value4 = simpleKalmanFilter4.updateEstimate(real_value4);

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) {
    Serial.print(estimated_value1,4);
    Serial.print(",");
    Serial.print(estimated_value2,4);
    Serial.print(",");
    Serial.print(estimated_value3,4);
    Serial.print(",");
    Serial.print(estimated_value4,4);
    Serial.println();
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

  #if DISTANCE
  dist=sr04.Distance();
  Serial.print("dist: ");
  Serial.println(dist);
  #endif

  
  myMotor->setSpeed(motor_speed);
  myMotor2->setSpeed(motor_speed);

  //int combined_side_values = (estimated_value2 + estimated_value3)/2; //NVM; SEE BELOW
  int moveThreshold = 10;

  checkStates();

  if(spd < max_spd){
    if(millis()%5 == 0){
      spd++;
    }
    myMotor->setSpeed(spd);
    myMotor2->setSpeed(spd);
  }

  bool right_balanced = (estimated_value1 < estimated_value2+moveThreshold && estimated_value1 > estimated_value2-moveThreshold );
  bool left_balanced = (estimated_value1 < estimated_value3+moveThreshold && estimated_value1 > estimated_value3-moveThreshold );
  //If mostly balanced,
  
  if (right_balanced && left_balanced) {
  //    Don't move
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE); 
    current_direction = OFF;
  }
//  If front greatest
  else if(estimated_value1 > estimated_value2 && estimated_value1 > estimated_value3 /*&& dist > 50 */) {
//    Go FWD
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    current_direction=FWD;
  }
//  Directions
  else {
    if (estimated_value2 > estimated_value3) {
      myMotor->run(FORWARD);
      myMotor2->run(BACKWARD);
      current_direction = RIGHT;
    } else {
      myMotor->run(BACKWARD);
      myMotor2->run(FORWARD);
      current_direction = LEFT;
    }
  }

  //delay(3000);

//// sleepydog stuff
  digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
  int sleepMS = Watchdog.sleep(5000);
////    // Code resumes here on wake.

  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
//
//  // Try to reattach USB connection on "native USB" boards (connection is
//  // lost on sleep). Host will also need to reattach to the Serial monitor.
//  // Seems not entirely reliable, hence the LED indicator fallback.
  #ifdef USBCON
////    USBDevice.attach();
  #endif
//
  Serial.print("I'm awake now! I slept for ");
  Serial.print(sleepMS, DEC);
  Serial.println(" milliseconds.");
  Serial.println();
}


void checkStates(){
  if(current_direction != prev_direction){
      spd = 0;
      prev_direction = current_direction;
  }
}
