#include <RTClib.h>
//test
const int trigPin[] = {4,12,18,2};
const int echoPin[] = {16,14,19,15};
double distance[4];

void setup() {
  // put your setup code here, to run once:
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
}
/*
double time(){
DateTime now = rtc.now();
tijdNu = no
w.
}

}
*/
