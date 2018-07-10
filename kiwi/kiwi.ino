#define BLYNK_PRINT Serial
 
#include <ESP8266WiFi.h>

#include <BlynkSimpleEsp8266.h>

#include <math.h>

// You should get Auth Token in the Blynk App.

// Go to the Project Settings (nut icon).

char auth[] = "ea85e0f14b56459e93cf7022f973764c";
 
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Fireball";
char pass[] = "change this later";

//int LED = D8; // Define LED as an Integer (whole numbers) and pin D8 on Wemos D1 Mini Pro

//L293D
//Motor Front
const int F1  = D2;  // Pin 14 of L293
const int F2  = D1;  // Pin 10 of L293
//Motor Right
const int R1  = D7; // Pin  7 of L293
const int R2  = D8;  // Pin  2 of L293
//Motor Left
const int L1 = D6;
const int L2 = D5;
int last_x = 0;
int last_y = 0;


int pwmMax = 1023;
void setup()
{
  // Debug console
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  pinMode(F1, OUTPUT);
  pinMode(F2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT); 
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
}

void loop()
{
  Blynk.run();
  bool result = Blynk.connected();
    //Calculate magnitude
  float mag = sqrt(pow(last_x,2)+ pow(last_y,2));
  if (mag <30){
    stopM();
  }
  else{
    //Caluclate angle
    float angle = atan2(double(last_y), double(last_x));
    //Serial.println(rad2deg(angle));
    float angleF = angle - deg2rad(90.0);
    //Serial.println(rad2deg(angleF));
    float angleR = angle + deg2rad(30.0);
    float angleL = angle + deg2rad(150.0);
    //Serial.println(rad2deg(angleL));
  
    setPower(F1, F2, angleF);
    setPower(L1, L2, angleL);
    setPower(R1, R2, angleR);
  }
  
 
}
 
// This function will be called every time button Widget
// in Blynk app writes values to the Virtual Pin V3
BLYNK_WRITE(V1) // Widget WRITEs to Virtual Pin V1
{   
  int x = param[0].asInt() - 511; // getting first value
  int y = param[1].asInt() - 511; // getting second value
  last_x = x;
  last_y = y;
//  //Calculate magnitude
//  float mag = sqrt(pow(x,2)+ pow(y,2));
//  if (mag <10){
//    stopM();
//  }
//  //Caluclate angle
//  float angle = atan2(double(y), double(x));
//  Serial.println(rad2deg(angle));
//  float angleF = angle - rad2deg(90.0);
//  float angleR = angle - rad2deg(210.0);
//  float angleL = angle + rad2deg(30.0);
//  
//  //setPower(F1, F2, angleF);
//  //setPower(L1, L2, angleL);
//  //setPower(R1, R2, angleR);
//  
//  
}

void setPower(int M1, int M2, float angle)
{
  //Serial.println(angle);
  float power = int(((sin(angle))/1.0)*1023.0);
  Serial.println(power);
  setMotor(M1, M2, power);
  
}
float deg2rad(float inputAngle)
{
  return inputAngle * (M_PI / 180.0);
}
float rad2deg(float inputAngle)
{
  return inputAngle * (180.0 / M_PI);
}
//{
//  if (speed > pwmMax){
//    speed = pwmMax;
//  }
//  else if (speed < 0){
//    speed = 0;
//  }
//  analogWrite(LF, speed);
//  analogWrite(RF, speed);
//  digitalWrite(LR, LOW);
//  digitalWrite(RR, LOW);
//}
//
//void rev(int speed){
//  if (speed > pwmMax){
//    speed = pwmMax;
//  }
//  else if (speed < 0){
//    speed = 0;
//  }
//  analogWrite(LR, speed);
//  analogWrite(RR, speed);
//  digitalWrite(LF, LOW);
//  digitalWrite(RF, LOW);
//}
//
//void cw(int speed){
//  if (speed > pwmMax){
//    speed = pwmMax;
//  }
//  else if (speed < 0){
//    speed = 0;
//  }
//  analogWrite(LF, speed);
//  analogWrite(RR, speed);
//  digitalWrite(LR, LOW);
//  digitalWrite(RF, LOW);
//}
//
//void ccw(int speed){
//  if (speed > pwmMax){
//    speed = pwmMax;
//  }
//  else if (speed < 0){
//    speed = 0;
//  }
//  analogWrite(RF, speed);
//  analogWrite(LR, speed);
//  digitalWrite(RR, LOW);
//  digitalWrite(LF, LOW);
//}
//
void stopM(){
    digitalWrite(F1, LOW);
    digitalWrite(F2, LOW);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);   
}

void setMotor(int M1, int M2, int level){
  if (level < 0){
    int temp = M1;
    M1 = M2;
    M2 = temp;
    level = -1 * level;
  }
  //Serial.println(level);
  analogWrite(M1, level);
  digitalWrite(M2, LOW);
}

