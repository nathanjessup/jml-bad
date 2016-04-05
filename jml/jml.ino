//GPS & SD
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

// Accel / Gyro / Mag
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();


//Matrix
#include <Wire.h>
#include <Adafruit_LEDBackpack.h>

//neopxiels
byte ledCount = 16;
byte neoPixelPin = 44;
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel lightBar = Adafruit_NeoPixel(ledCount, neoPixelPin, NEO_GRB + NEO_KHZ800);
uint32_t currentColor;
//Some nice colors to choose from
uint32_t red;
uint32_t blue;
uint32_t green;
uint32_t white;
byte lightBarBrightness = 255;

//GPS
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false
#define LOG_FIXONLY true  
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


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


uint16_t maxIdleRpm = 2000;
uint16_t powerBandRpm = 6800;
uint16_t shiftLightRpm = 11500;

byte ignitionPulses = 0;
signed int rpm = 0;

boolean demo = true;
signed int demoDelta = 45;

void initPins(){
    pinMode(10, OUTPUT);
}

//setup the light bar
void initLightBar() {
  //Setup some default colors
  initColors();

  lightBar.begin();
  lightBar.setBrightness(lightBarBrightness);
  writeLightBar();
}

void initColors() {
  red = lightBar.Color(255, 0, 0);
  blue = lightBar.Color(0, 0, 255);
  green = lightBar.Color(0, 255, 0);
  white = lightBar.Color(255, 255, 255);
  //magenta = lightBar.Color(255, 0, 255);
  currentColor = blue;
}

void initLogger(){
  if (!SD.begin(chipSelect, 11, 12, 13)) {
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
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
  
    useInterrupt(true);
}

void initSegDisplay() {
  matrix.begin(0x70);
  matrix.setBrightness(matrixBrightness);
}

void initLSM()
{
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void pinThreeInterupt() {
  ignitionPulses++;
}

void writeLightBar() {

  if(rpm < maxIdleRpm){
    currentColor = green;
  } 
  else if( rpm < powerBandRpm) {
    currentColor = blue;
  } 
  else if( rpm < shiftLightRpm ) {
    currentColor = red;
  } 
  else if( rpm > shiftLightRpm ) {
    currentColor = white;
  }
  byte ledsToLight = rpm / ((shiftLightRpm / ledCount)) >= 1 ? rpm / ((shiftLightRpm / ledCount)) : 1;
  for(int i = 0; i < ledCount; i++){
    if(i < ledsToLight) {
      lightBar.setPixelColor(i, currentColor);
    } 
    else {
      lightBar.setPixelColor(i, 0,0,0); //off
    }
  };
  lightBar.show();
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
    } else if(GPSECHO) {
        Serial.println(stringptr);
    }
    uint8_t stringsize = strlen(stringptr);
    stringsize != logfile.print(stringptr);  //write the string to the SD file
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void readLSM(){
  
   sensors_event_t accel, mag, gyro, temp;
   lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
   // print out accelleration data
   // silly lib is off by an order of mag on teh data... it will report .98 m/s^2 instead of 9.8 m/s^2
  logfile.print(accel.acceleration.x * 10 ); logfile.print(",");
  logfile.print(accel.acceleration.y * 10 );       logfile.print(",");
  logfile.print(accel.acceleration.z * 10 );     logfile.print(",");
 
  // print out magnetometer data
  logfile.print(mag.magnetic.x); logfile.print(",");
  logfile.print(mag.magnetic.y);       logfile.print(",");
  logfile.print(mag.magnetic.z);     logfile.print(",");
  
  // print out gyroscopic data
  logfile.print(gyro.gyro.x); logfile.print(",");
  logfile.print(gyro.gyro.y);       logfile.print(",");
  logfile.print(gyro.gyro.z);     logfile.print(",");
  
  // temp
  logfile.println(temp.temperature);
  
  //lsm.read();

//  logfile.print((int)lsm.accelData.x);      logfile.print(",");
//  logfile.print((int)lsm.accelData.y);        logfile.print(",");
//  logfile.print((int)lsm.accelData.z);       logfile.print(",");
//  logfile.print((int)lsm.magData.x);       logfile.print(",");
//  logfile.print((int)lsm.magData.y);          logfile.print(",");
//  logfile.print((int)lsm.magData.z)  ;         logfile.print(",");
//  logfile.print((int)lsm.gyroData.x);     logfile.print(",");
//  logfile.print((int)lsm.gyroData.y);        logfile.print(",");
//  logfile.print((int)lsm.gyroData.z); 
//  logfile.print("\n");
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
  Serial.begin(57600);
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
   Serial.println(F("init lsm"));
  initLSM();
  initLightBar();
  //pin 2 is interupt 0 pin 3 is interupt 1
  //attachInterrupt(0, pinTwoInterupt, LOW);
  attachInterrupt(1, pinThreeInterupt, RISING);  //this is the ignition interupt
  Serial.println(F("end setup"));
}

void loop()  {
  
  currentTime = millis();
  //readGPS();
  //if(currentTime >= (loopTime + dataLoopInterval )){
//    readGPS();
//    readLSM();
//    loopTime = currentTime;
//  }
  if(currentTime >= (displayTime + renderLoopInterval )){
    readGPS();
    logfile.print(currentTime); logfile.print(",");
    readLSM();
    logfile.flush();
    //if(demo) doDemo();
    //writeMatrix(rpm);
    //writeLightBar();
    displayTime = currentTime;
  }
}
