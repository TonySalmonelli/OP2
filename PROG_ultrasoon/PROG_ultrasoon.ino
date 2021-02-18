#include <RTClib.h>
//test
const int trigPin[] = {0,15,18,14};
const int echoPin[] = {4,2,19,5};
double distance[4];

void setup() {
  // put your setup code here, to run once:
  /*
  for(int t=0;t<2;t++){
    pinMode(trigPin[t],OUTPUT);
    for(int j = t*2; t<(t*2)+2;t++)
    {
      pinMode(echoPin[t+j],INPUT);
    }
  }
  */

  for(int t=0;t<4;t++){
    pinMode(trigPin[t],OUTPUT);
    pinMode(echoPin[t],INPUT);
    
  }
  Serial.begin(9600);
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  sonicsensor();
  
  Serial.print(distance[0]);
  
  Serial.print("\t");
  Serial.print(distance[1]);
  Serial.print("\t");
  Serial.print(distance[2]);
  Serial.print("\t");
  Serial.println(distance[3]);
    //Serial.println(duration);
    
/*
  switch():
    case 0: 
      
*/
}
void sonicsensor(){
  long duration;
  
  for(int t=0;t<4;t++){
    digitalWrite(trigPin[t], LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin[t],HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[t],LOW);

    duration = pulseIn(echoPin[t],HIGH);
    distance[t] = duration*0.034/2;
    delay(200);
  }
  
  /*
  for(int t=0;t<2;t++){
    digitalWrite(trigPin[t], LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin[t],HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[t],LOW);

    for(int j = t * 2; j < (t * 2) + 2; j++)
    {
      duration = pulseIn(echoPin[(j)],HIGH);
      distance[j] = duration*0.034/2;
    }
    delay(200);
    */
  
}
/*
double time(){
DateTime now = rtc.now();
tijdNu = no
w.
}

}
*/
