void driveSmallDistance(int directions){
  driving(directions);
  delay(1000);
  brakes();
}

void driveLeft(){
  if(distance[0] < 20){
    leftDir(120);
  }
  else{
    forward(120);
  }
}
