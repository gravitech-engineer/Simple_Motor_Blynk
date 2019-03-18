/*************************************************************
   Name: Car_Control_Blynk
   Device: KidBrigth32 V1.3
   Chip: ESP32
   Compile: Node32s
   Create: 25-5-18
*************************************************************/
#define BLYNK_PRINT Serial // Comment this out to disable prints and save space
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "KB_Motor.h"

/*********************************************************************
   Constant: Dot Matrix Define
 *********************************************************************/

static const uint8_t PROGMEM
    UP1[] =
        {B11111111,
         B01111111,
         B00110000,
         B00011000,
         B00001100,
         B00000000,
         B00000000,
         B00000000},
    UP2[] =
        {B00000000,
         B00000000,
         B00000000,
         B00001100,
         B00011000,
         B00110000,
         B01111111,
         B11111111},
    L1[] =
        {B00111100,
         B11111111,
         B01111110,
         B00111100,
         B00011000,
         B00000000,
         B00000000,
         B00000000},
    L2[] =
        {B00000000,
         B00000000,
         B00000000,
         B00111100,
         B00111100,
         B00111100,
         B00111100,
         B00111100},
    R1[] =
        {B00111100,
         B00111100,
         B00111100,
         B00111100,
         B00111100,
         B00000000,
         B00000000,
         B00000000},
    R2[] =
        {B00000000,
         B00000000,
         B00000000,
         B00011000,
         B00111100,
         B01111110,
         B11111111,
         B00111100},
    BW1[] =
        {B11111111,
         B11111110,
         B00001100,
         B00011000,
         B00110000,
         B00000000,
         B00000000,
         B00000000},
    BW2[] =
        {B00000000,
         B00000000,
         B00000000,
         B00110000,
         B00011000,
         B00001100,
         B11111110,
         B11111111},
    ST1[] =
        {B10111101,
         B10111101,
         B01000010,
         B00111100,
         B00000000,
         B00000000,
         B00000000,
         B00000000},
    ST2[] =
        {B00000000,
         B00000000,
         B00000000,
         B00000000,
         B00111100,
         B01000010,
         B10111101,
         B10111101};


Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
KB_MOTOR i2c_motor; //Create a new KB_MOTOR instance

uint8_t motor1_addr = 0x66; //i2c Address Motor Control Motor 1
uint8_t motor2_addr = 0x68; //i2c Address Motor Control Motor 2


#define BLYNK_PRINT Serial
//Blynk auth
char auth[] = "4691d58d92cb46eab75681969863c6d8";
//SSID & PASS for network
char ssid[] = "GT-BOOTH";
char pass[] = "27/14GtechRICH";

void setup()
{
  Serial.begin(9600);
  if (Wire1.begin(4, 5)) //I2C2
    Serial.println("Wire 1 : begin");
  if (Wire.begin(21, 22)) //default I2C1
    Serial.println("Wire 0 : begin");

  Blynk.begin(auth, ssid, pass);
}

void loop()
{
  Blynk.run();
}

//i2c_motor.i2c_motor_write(1, 127, 2);
//i2c_motor.i2c_motor_write(CH, SPEED, State); CH = 1,2  :  Speed = 1-255  :  State = 1 -> FWD , 2 BWD , 0 -> Stop
//Robot Driving Functions
void Robot_Forward()
{
  i2c_motor.begin(motor1_addr, motor2_addr);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.i2c_motor_write(1, 255, 1);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.faultCheck2(motor2_addr);
  i2c_motor.i2c_motor_write(2, 255, 1);
  i2c_motor.faultCheck2(motor2_addr);
  Wire1.endTransmission();
}
void Robot_Backward()
{
  i2c_motor.begin(motor1_addr, motor2_addr);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.i2c_motor_write(1, 255, 2);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.faultCheck2(motor2_addr);
  i2c_motor.i2c_motor_write(2, 255, 2);
  i2c_motor.faultCheck2(motor2_addr);
  Wire1.endTransmission();
}
void Robot_Left()
{
  i2c_motor.begin(motor1_addr, motor2_addr);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.i2c_motor_write(1, 120, 2);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.faultCheck2(motor2_addr);
  i2c_motor.i2c_motor_write(2, 120, 1);
  i2c_motor.faultCheck2(motor2_addr);
  Wire1.endTransmission();
}
void Robot_Right()
{
  i2c_motor.begin(motor1_addr, motor2_addr);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.i2c_motor_write(1, 120, 1);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.faultCheck2(motor2_addr);
  i2c_motor.i2c_motor_write(2, 120, 2);
  i2c_motor.faultCheck2(motor2_addr);
  Wire1.endTransmission();
}

void Robot_Stop()
{
  i2c_motor.begin(motor1_addr, motor2_addr);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.i2c_motor_write(1, 255, 0);
  i2c_motor.faultCheck1(motor1_addr);
  i2c_motor.faultCheck2(motor2_addr);
  i2c_motor.i2c_motor_write(2, 255, 0);
  i2c_motor.faultCheck2(motor2_addr);
  Wire1.endTransmission();
}

void Display_FWD()
{
  matrix.begin(0x70);
  Serial.println("FWD");
  matrix.setRotation(0);
  matrix.clear();
  matrix.drawBitmap(0, 8, BW1, 8, 8, LED_ON);
  matrix.writeDisplay();
  matrix.drawBitmap(0, 0, BW2, 8, 8, LED_ON);
  matrix.writeDisplay();
  Wire.endTransmission();
}

void Display_L()
{
  matrix.begin(0x70);
  Serial.println("L");
  matrix.setRotation(0);
  matrix.clear();
  matrix.drawBitmap(0, 8, R1, 8, 8, LED_ON);
  matrix.writeDisplay();
  matrix.drawBitmap(0, 0, R2, 8, 8, LED_ON);
  matrix.writeDisplay();
  Wire.endTransmission();
}

void Display_R()
{
  matrix.begin(0x70);
  Serial.println("R");
  matrix.setRotation(0);
  matrix.clear();
  matrix.drawBitmap(0, 8, L1, 8, 8, LED_ON);
  matrix.writeDisplay();
  matrix.drawBitmap(0, 0, L2, 8, 8, LED_ON);
  matrix.writeDisplay();
  Wire.endTransmission();
}

void Display_BWD()
{
  matrix.begin(0x70);
  Serial.println("BWD");
  matrix.setRotation(0);
  matrix.clear();
  matrix.drawBitmap(0, 8, UP1, 8, 8, LED_ON);
  matrix.writeDisplay();
  matrix.drawBitmap(0, 0, UP2, 8, 8, LED_ON);
  matrix.writeDisplay();
  Wire.endTransmission();
}

void Display_STP()
{
  matrix.begin(0x70);
  matrix.setRotation(0);
  matrix.clear();
  matrix.drawBitmap(0, 8, ST1, 8, 8, LED_ON);
  matrix.writeDisplay();
  matrix.drawBitmap(0, 0, ST2, 8, 8, LED_ON);
  matrix.writeDisplay();
  Wire.endTransmission();
}

BLYNK_WRITE(V3)
{
  int value = param.asInt(); // Get value as integer
  Serial.println("Going Forward");
  if (value)
  {
    Robot_Forward();
    Display_BWD();
  }
}

BLYNK_WRITE(V2)
{
  int value = param.asInt(); // Get value as integer
  Serial.println("Moving Right");
  if (value)
  {
    Robot_Right();
    delay(200);
    Robot_Stop();
    Display_L();
  }
}

BLYNK_WRITE(V1)
{
  int value = param.asInt(); // Get value as integer
  Serial.println("Going back");
  if (value)
  {
    Robot_Backward();
    Display_FWD();
  }
}

BLYNK_WRITE(V4)
{
  int value = param.asInt(); // Get value as integer
  Serial.println("Taking Left");
  if (value)
  {
    Robot_Left();
    delay(200);
    Robot_Stop();
    Display_R();
  }
}

BLYNK_WRITE(V5)
{
  int value = param.asInt(); // Get value as integer
  Serial.println("Braking!!");
  if (value)
  {
    Robot_Stop();
    Display_STP();
  }
}
