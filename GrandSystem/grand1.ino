#include <SPI.h>
#include <SD.h>
#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if(!Serial.available()){
    ;
  }
}

void loop(){
  int count=0;
  File data = SD.open("log.txt",FILE_WRITE);
  if(Serial.available()){
    data.print(Serial.read());
    data.println("");
    count = 0;
  }else{
    count++;
  }
  if(count>=1000){
    data.close();
  }
}
