#include <Wire.h>
#include <Adafruit_LEDBackpack.h>


Adafruit_7segment matrix = Adafruit_7segment();
byte matrixBrightness = 1;

byte dataLoopInterval = 10;
byte renderLoopInterval = 100;
unsigned long currentTime;
unsigned long loopTime;
unsigned long displayTime;


boolean demo = true;
signed int demoDelta = 45;
signed int rpm = 0;

void initSegDisplay() {
   Serial.print("initSeg");
  matrix.begin(0x70);
  matrix.setBrightness(matrixBrightness);
   Serial.print("initSegEnd");
}
void writeMatrix(int val) {
  if(val <= 9999){
    matrix.print(val,DEC);
  } 
  else {
    matrix.writeDigitNum(0, (val / 10000));
    matrix.writeDigitNum(1, (val / 1000) % 10,true);
    matrix.writeDigitNum(3, (val / 100) % 10);
    matrix.writeDigitNum(4, (val / 10) % 10);
  }
  matrix.writeDisplay();
}

void doDemo(){
//  Serial.print("rpm ");
//  Serial.println(rpm);
//  Serial.print("demoDelta ");
//  Serial.println(rpm + demoDelta);
  
    if(rpm + demoDelta <= 14000 && rpm + demoDelta >= 0){
      rpm = rpm + demoDelta;
    }else{
      //flip the sign
      demoDelta = demoDelta * -1;
    }
    S
}

void setup() {
  Serial.begin(115200);
  Serial.println("ss");
  initSegDisplay();
  Serial.println("es");
}

void loop()  {
  
  currentTime = millis();
  if(currentTime >= (loopTime + dataLoopInterval )){}
  if(currentTime >= (displayTime + renderLoopInterval )){
    if(demo) doDemo();
    writeMatrix(rpm);
    displayTime = currentTime;
  }
}
