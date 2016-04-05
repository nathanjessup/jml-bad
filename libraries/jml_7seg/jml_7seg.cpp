#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

#include "jml_7seg.h"


Jml_7seg::Jml_7seg(void){
  Adafruit_7segment matrix = Adafruit_7segment();
}

int Jml_7seg::write(int num) {
  if(num <= 9999){
    matrix.print(num,DEC);
  } else {
    matrix.writeDigitNum(0, (num / 10000));
    matrix.writeDigitNum(1, (num / 1000) % 10,true);
    matrix.writeDigitNum(3, (num / 100) % 10);
    matrix.writeDigitNum(4, (num / 10) % 10);
  }
  matrix.writeDisplay();
}

void Jml_7seg::init(){
  matrix.begin(0x70);
  matrix.setBrightness(1);
  Jml_7seg::write(1337);
}