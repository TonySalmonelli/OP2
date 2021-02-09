

#define joyx 34
#define joyy 35
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int xValue = analogRead(joyx);
  int yValue = analogRead(joyy);
  int xMap = map(xValue, 0,4096, 0, 255);
  int yMap = map(yValue,0,4096,255,0);
  if(89 < yMap > 255){
    //links
  }
  if(0 < yMap > 70){
    //rechts
  }
  if(175 < xMap && xMap > 255){
    // vooruit
  }
  if(0 < xMap > 150){
    //achteruit
  }
  if(151 < xMap > 175 && 71 < yMap > 90){
    //stoppen
  }
  //print the values with to plot or view
  Serial.print(xMap);
  Serial.print("\t");
  Serial.println(yMap);
}
