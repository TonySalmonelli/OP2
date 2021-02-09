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


  Serial.print(stateLB);
  Serial.print("\t");
  Serial.println(stateRB);
  Serial.println(potValue);
  } // end loop
  
//functions movements motors
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
