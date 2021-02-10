#include <Wire.h>
float yaw,roll,pitch,accx,accy,accz,gyrox,gyroy,gyroz,x0gy,y0gy,z0gy,xgy,ygy,zgy;
const int GY_BNO05=0x29;

// drive motor (Nr, Forwards, backwards)zal
#define mot1F 25
#define mot1B 26
#define mot2F 32
#define mot2B 33
#define mot3F 27
#define mot3B 23
#define mot4F 17
#define mot4B 16

#define LB 12 // Left turn Button
#define RB 14 // Right turn Button
#define potPin 13

int stateLB = 0;
int stateRB = 0;
int potValue;
double motorVoltage; //Changing velocity of motor
#define joyx 34 // joystick x-value raw
#define joyy 35 // joystick y-value raw
//pin 2 leftDir
//pin 0 rightDir

int currentYaw;
int BeginYaw;

void setup() {
  
  // setup motor pins
  ledcAttachPin(mot1F,1);
  ledcAttachPin(mot1B,2);
  ledcAttachPin(mot2F,3);
  ledcAttachPin(mot2B,4);
  ledcAttachPin(mot3F,5);
  ledcAttachPin(mot3B,6);
  ledcAttachPin(mot4F,7);
  ledcAttachPin(mot4B,8);
  ledcSetup(1,5000,8);
  ledcSetup(2,5000,8);
  ledcSetup(3,5000,8);
  ledcSetup(4,5000,8);
  ledcSetup(5,5000,8);
  ledcSetup(6,5000,8);
  ledcSetup(7,5000,8);
  ledcSetup(8,5000,8);


  //setup gyroscoop
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  Wire.beginTransmission(GY_BNO05);
  Wire.write(0x3E); // Power Mode
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(GY_BNO05);
  Wire.write(0x3D); // Operation Mode
  Wire.write(0x0C); //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(100);
  
  BeginYaw = gyroWaarde(1);
  
  Serial.begin(9600);

  pinMode(LB, INPUT);
  pinMode(RB, INPUT);

  
}

void loop() {
  //changing velocity with potmeter
  potValue = analogRead(potPin);
  motorVoltage = potValue/16;
  // LB, RB read
  stateLB = digitalRead(LB);
  stateRB = digitalRead(RB);
  //mapping joystick
  int xValue = analogRead(joyx);
  int yValue = analogRead(joyy);
  int xMap = map(xValue, 0,4096, 0, 255);
  int yMap = map(yValue,0,4096,255,0);
  
  if(yMap > 150){
    leftDir(motorVoltage);
  }
  else if(yMap < 120){
    rightDir(motorVoltage);
  }
  else if(xMap > 120){
    forward(motorVoltage);
  }
  else if(xMap < 100){
    backwards(motorVoltage);
  }
  else if(stateLB == 1){
    leftTurn(motorVoltage);
  }
  else if( stateRB == 1){
    rightTurn(motorVoltage);
  }
  else {
    brakes();
  }

  currentYaw = gyroWaarde(1);
  Serial.println(BeginYaw);
  Serial.println(currentYaw);
  blijfRecht();

  //Serial.print(stateLB);
  //Serial.print("\t");
  //Serial.println(stateRB);
  //Serial.println(potValue);
  
  } // end loop
  
//+++++++++++++++++++++++++++++++++++++++functions++++++++++++++++++++++++++ 
//functions movements motors----------------------------------------------
void forward(int power){

  ledcWrite(1,power);
  ledcWrite(2,0);
  ledcWrite(3,power);
  ledcWrite(4,0);
  ledcWrite(5,power);
  ledcWrite(6,0);
  ledcWrite(7,power);
  ledcWrite(8,0);
}

void backwards(int power){

  ledcWrite(1,0);
  ledcWrite(2,power);
  ledcWrite(3,0);
  ledcWrite(4,power);
  ledcWrite(5,0);
  ledcWrite(6,power);
  ledcWrite(7,0);
  ledcWrite(8,power);
}

void leftDir(int power)
{
  ledcWrite(1,0);
  ledcWrite(2,power);
  ledcWrite(3,power);
  ledcWrite(4,0);
  ledcWrite(5,0);
  ledcWrite(6,power);
  ledcWrite(8,0);
  ledcWrite(7,power);
}
void rightDir(int power)
{
  ledcWrite(2,0);
  ledcWrite(1,power);
  ledcWrite(4,power);
  ledcWrite(3,0);
  ledcWrite(6,0);
  ledcWrite(5,power);
  ledcWrite(7,0);
  ledcWrite(8,power);
}
void brakes()
{ 
  ledcWrite(1,0);
  ledcWrite(2,0);
  ledcWrite(3,0);
  ledcWrite(4,0);
  ledcWrite(5,0);
  ledcWrite(6,0);
  ledcWrite(7,0);
  ledcWrite(8,0);
}
void rightTurn(int power)
{
  ledcWrite(1,0);
  ledcWrite(2,power);
  ledcWrite(4,0);
  ledcWrite(3,power);
  ledcWrite(6,0);
  ledcWrite(5,power);
  ledcWrite(7,0);
  ledcWrite(8,power);
}
void leftTurn(int power)
{
  ledcWrite(2,0);
  ledcWrite(1,power);
  ledcWrite(3,0);
  ledcWrite(4,power);
  ledcWrite(5,0);
  ledcWrite(6,power);
  ledcWrite(8,0);
  ledcWrite(7,power);
}

//gyroscoop-------------------------------------------------------------

int gyroWaarde(int index)
{
  Wire.beginTransmission(GY_BNO05);
  Wire.write(0x08); 
  Wire.endTransmission(false);
  Wire.requestFrom(GY_BNO05,32,true);
// Accelerometer
  accx=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accy=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accz=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
// Magnetometer
  (Wire.read()|Wire.read()<<8 ); 
  (Wire.read()|Wire.read()<<8 ); 
  (Wire.read()|Wire.read()<<8 ); 
// Gyroscope
  gyrox=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroy=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroz=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
// Euler Angles
  yaw=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
  roll=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
  pitch=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00;  //in Degrees unit
// Quaternions
  (Wire.read()|Wire.read()<<8 );
  (Wire.read()|Wire.read()<<8 );
  (Wire.read()|Wire.read()<<8 );
  (Wire.read()|Wire.read()<<8 );

  //kijk hier welk getal je nodig hebt voor je gyro waarde
  switch(index)
  {
    case 1:
      return yaw;
      break;
    case 2:
      return roll;
      break;
    case 3:
      return pitch;
      break;
    default:
      return 0;
      break;
  }
}
void blijfRecht()
{
  int allowAngle = 15;
  int adjustValue = 150;
  if(currentYaw <= 360 + BeginYaw - allowAngle && currentYaw >= 180 + BeginYaw )
  {
     rightTurn(adjustValue);
     delay(100);
  }
  if(currentYaw >= BeginYaw + allowAngle && currentYaw <= 180+ BeginYaw)
  {
    leftTurn(adjustValue);
    delay(100);
  }
}
