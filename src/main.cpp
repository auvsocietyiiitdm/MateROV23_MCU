#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <FXAS21002C_Basic.h>
#include <FXOS8700Q_Basic.h>
#include <MadgwickAHRS.h>
#include <vector>

#define NO_OF_THRUSTERS 7
#define MAX_FORWARD_THRUST_PWM 1700
#define MAX_BACKWARD_THRUST_PWM 1300
#define ZERO_THRUST_PWM 1500
#define SDA_PIN PB7
#define SCL_PIN PB6
#define ACC_MAG_ADDRESS 0x1F
#define GYRO_ADDRESS 0x21
const uint8_t thruster_pins[NO_OF_THRUSTERS] = {PA0, PA1, PA2, PA3, PA6, PA7, PB0};

// User defined functions
void setThrusterSpeed(Servo* thruster, int pwm_value);
void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values);
void log_IMU_data(float *Acc, float *Mag, float *Gyr);

// Global variables for thruster control
std::vector<Servo> thrusters;
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub("pwm_values", &pwm_values_cb);
// Global variables for sensor data collection
TwoWire wire(SDA_PIN, SCL_PIN);
FXAS21002CBasic Gyro;
FXOS8700QBasic AcMg;
Madgwick Filter;
float Acc[3], Gyr[3], Mag[3];
long prevTime = millis();
char buffer[100];


void setup(){
  // Creating thruster control vector
  for(int i=0; i<NO_OF_THRUSTERS; i++){
    Servo thruster;
    thrusters.push_back(thruster);
    thrusters[i].attach(thruster_pins[i]);
  }
  // Initializing the thrusters
  for(int i=0; i<NO_OF_THRUSTERS; i++){
    setThrusterSpeed(&thrusters[i], 1500);
  }
  delay(5000);

  // Initialization for sensor data collection
  wire.begin();
  Gyro = FXAS21002CBasic(GYRO_ADDRESS, &wire);
  AcMg = FXOS8700QBasic(1, ACC_MAG_ADDRESS, &wire);
  delay(100);

  // Initializing ROS
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  // Log IMU data every 1sec
  if(millis() - prevTime > 1000){
  Gyro.updateGyroData(&Gyr[0]);
  AcMg.updateAccelMagData(&Acc[0], &Mag[0]);
  log_IMU_data(&Acc[0], &Mag[0], &Gyr[0]);
  prevTime = millis();
  }
  // Control thrusters everytime pwm_values are published
  nh.spinOnce();
}

void setThrusterSpeed(Servo* thruster, int pwm_value){
  // For reverse thrust
  if((pwm_value < ZERO_THRUST_PWM) && (pwm_value >= MAX_BACKWARD_THRUST_PWM)){
    for(int i = ZERO_THRUST_PWM; i >= pwm_value; i -= 10){
      thruster->writeMicroseconds(i);
    }
  }
  // For forward thrust
  else if((pwm_value > ZERO_THRUST_PWM) && (pwm_value <= MAX_FORWARD_THRUST_PWM)){
    for(int i = ZERO_THRUST_PWM; i <= pwm_value; i += 10){
      thruster->writeMicroseconds(i);
    }
  }
  // For zero thrust / initialization
  else if(pwm_value == ZERO_THRUST_PWM){
    for(int i = ZERO_THRUST_PWM - 20; i <= ZERO_THRUST_PWM; i += 10){
      thruster->writeMicroseconds(i);
    }
  }
}

void pwm_values_cb(const std_msgs::Int32MultiArray& pwm_values){
  for(int i = 0; i < NO_OF_THRUSTERS; i++){
    setThrusterSpeed(&thrusters[i], pwm_values.data[i]);
  }
}

void log_IMU_data(float *Acc, float *Mag, float *Gyr){
  char acc0[5], acc1[5], acc2[5], gyr0[5], gyr1[5], gyr2[5], mag0[5], mag1[5], mag2[5];
  dtostrf(Acc[0],4,2,acc0); dtostrf(Acc[1],4,2,acc1); dtostrf(Acc[2],4,2,acc2);
  dtostrf(Gyr[0],4,2,gyr0); dtostrf(Gyr[1],4,2,gyr1); dtostrf(Gyr[2],4,2,gyr2);
  dtostrf(Mag[0],4,2,mag0); dtostrf(Mag[1],4,2,mag1); dtostrf(Mag[2],4,2,mag2);
  sprintf(buffer, "Acc: %s , %s, %s", acc0, acc1, acc2);
  nh.loginfo(buffer);
  sprintf(buffer, "Gyro: %s , %s , %s",gyr0, gyr1, gyr2);
  nh.loginfo(buffer);
  sprintf(buffer, "Mag: %s , %s , %s\n", mag0, mag1, mag2);
  nh.loginfo(buffer);
}