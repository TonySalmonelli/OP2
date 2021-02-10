#include <RTClib.h>

const int trigPin = 4;
const int echoPin= 16;

long duration;
double distance;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:

//RTC_DS1307 rtc;
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

digitalWrite(trigPin,HIGH);
delayMicroseconds(10);
digitalWrite(trigPin,LOW);

duration = pulseIn(echoPin,HIGH);
distance = duration*0.034/2;

Serial.println(""+String(distance) + "          ");
//Serial.println(duration);
delay(10);

}
/*
double time(){
DateTime now = rtc.now();
tijdNu = now.
}

}
*/
