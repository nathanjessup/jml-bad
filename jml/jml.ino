//GPS & SD
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

//Matrix
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>

//GPS
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true
#define LOG_FIXONLY false  

//SD
File logfile;
#define chipSelect 10 //sd uses pin 10

Adafruit_7segment matrix = Adafruit_7segment();
byte matrixBrightness = 1;

byte dataLoopInterval = 10;
byte renderLoopInterval = 100;
unsigned long currentTime;
unsigned long loopTime;
unsigned long displayTime;

byte ignitionPulses = 0;
signed int rpm = 0;

boolean demo = true;
signed int demoDelta = 45;

void initPins(){
    pinMode(10, OUTPUT);
}

void initLogger(){
 if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println(F("Card init. failed!"));
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print(F("Couldnt create ")); 
    Serial.println(filename);
  }
  Serial.print(F("Writing to ")); 
  Serial.println(filename); 
}

void initGPS(){
 // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
}

void initSegDisplay() {
  matrix.begin(0x70);
  matrix.setBrightness(matrixBrightness);
}

void pinThreeInterupt() {
  ignitionPulses++;
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

void readGPS(){
   // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
      
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    
    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Sentence parsed! 
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print(F("No Fix"));
      return;
    }
//    return stringprt; 
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        Serial.println(F("string to large"));
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
  }
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
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
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("start setup"));
    Serial.println(freeRam());
  initSegDisplay();
  Serial.println(freeRam());
   Serial.println(F("init logger"));
  initLogger();
  Serial.println(freeRam());
   Serial.println(F("init gps"));
  initGPS();
  Serial.println(freeRam());
  //pin 2 is interupt 0 pin 3 is interupt 1
  //attachInterrupt(0, pinTwoInterupt, LOW);
  attachInterrupt(1, pinThreeInterupt, RISING);  //this is the ignition interupt
  Serial.println(F("end setup"));
}

void loop()  {
  
  currentTime = millis();
  if(currentTime >= (loopTime + dataLoopInterval )){
    readGPS();
    loopTime = currentTime;
  }
  if(currentTime >= (displayTime + renderLoopInterval )){
    if(demo) doDemo();
    writeMatrix(rpm);
    displayTime = currentTime;
  }
}
