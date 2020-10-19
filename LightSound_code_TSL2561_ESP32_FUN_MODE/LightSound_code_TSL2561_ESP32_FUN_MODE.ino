/* Developed for the LightSound 2.0 */
/* Code developed by Soley Hyman, with help from Daniel Davis, Rob Hart, and Keith Crouch */
/* Modified for use with TSL2561 by Henry López del Pino */
/* last updated: 3 Sept 2019 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "SPIFFS.h"
#include "Player.h"
#include <SPI.h>

#define I2C_SDA 13
#define I2C_SCL 15

#define NEOPIXEL_PIN 14

#define VS_XCS    26 // Control Chip Select Pin (for accessing SPI Control/Status registers)
#define VS_XDCS   33 // Data Chip Select / BSYNC Pin
#define VS_DREQ   32 // Data Request Pin: Player asks for more data
#define SPI_SS    5 // Pin for enable SPI on some boards

#define VS_RESET  25 //Reset is active low

#define ESP32_BUTTON 14 //To change mode


// To change mode
int mode_lightsound = 0;
int old_value = 0;

// The band
Player player;


// I2C Init
TwoWire I2C_BUS = TwoWire(0);

int ind_global = 0;

byte percussion_time_array[] =  {0, 240, 480, 480, 480, 136, 104, 240, 480, 480, 480, 136, 104,
0, 240, 480, 480, 480, 136, 104, 240, 480, 480, 480, 136, 1784, 136,
0, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480,
0, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240,
0, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784,
0, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480,
0, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104,
0, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136,
0, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480,
0, 480, 480, 136, 24824, 136, 104, 240, 480, 480, 480, 136, 1784, 136,
0, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480,
0, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240,
0, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784,
0, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480,
0, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136, 104,
0, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136,
0, 1784, 136, 104, 240, 480, 480, 480, 136, 17144, 136, 104, 240, 480,
0, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480, 136, 1784, 136,
0, 104, 240, 480, 480, 480, 136, 1784, 136, 104, 240, 480, 480, 480,
0, 136};
byte percussion_array[] =   {76, 74, 69, 69, 69, 69, 76, 74, 69, 69, 69, 69, 76,
76, 74, 69, 69, 69, 69, 76, 74, 69, 69, 69, 69, 69, 69,
69, 74, 72, 69, 69, 67, 71, 64, 64, 76, 74, 69, 67, 69,
69, 64, 69, 69, 74, 72, 69, 69, 67, 71, 64, 64, 76, 74,
74, 69, 67, 69, 64, 69, 69, 74, 72, 69, 69, 67, 71, 64,
64, 64, 76, 74, 69, 67, 69, 64, 69, 69, 74, 72, 69, 69,
69, 67, 71, 64, 64, 76, 74, 69, 67, 69, 64, 69, 69, 74,
74, 72, 69, 69, 67, 71, 64, 64, 76, 74, 69, 67, 69, 64,
64, 69, 69, 74, 72, 69, 69, 67, 71, 64, 64, 76, 74, 69,
69, 67, 69, 64, 69, 69, 74, 72, 69, 69, 67, 71, 64, 64,
64, 76, 74, 69, 67, 69, 64, 69, 69, 74, 72, 69, 69, 67,
67, 71, 64, 64, 76, 74, 69, 67, 69, 64, 69, 69, 74, 72,
72, 69, 69, 67, 71, 64, 64, 76, 74, 69, 67, 69, 64, 69,
69, 69, 74, 72, 69, 69, 67, 71, 64, 64, 76, 74, 69, 67,
67, 69, 64, 69, 69, 74, 72, 69, 69, 67, 71, 64, 64, 76,
76, 74, 69, 67, 69, 64, 69, 69, 74, 72, 69, 69, 67, 71,
71, 64, 64, 76, 74, 69, 67, 69, 64, 69, 69, 74, 72, 69,
69, 69, 67, 71, 64, 64, 76, 74, 69, 67, 69, 64, 69, 69,
69, 74, 72, 69, 69, 67, 71, 64, 64, 76, 74, 69, 67, 69,
69, 64};

byte percussion_velocity_array[] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,
120, 120};
/**************************************************************************/

// Based on https://gist.github.com/microtherion/2636608 (MP3_Shield_RealtimeMIDI.ino from Matthias Neeracher)

//Write to VS10xx register
//SCI: Data transfers are always 16bit. When a new SCI operation comes in 
//DREQ goes low. We then have to wait for DREQ to go high again.
//XCS should be low for the full duration of operation.
void VSWriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte){
  while(!digitalRead(VS_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(VS_XCS, LOW); //Select control

  //SCI consists of instruction byte, address byte, and 16-bit data word.
  SPI.transfer(0x02); //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  while(!digitalRead(VS_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(VS_XCS, HIGH); //Deselect Control
}

//
// Plugin to put VS10XX into realtime MIDI mode
// Originally from http://www.vlsi.fi/fileadmin/software/VS10XX/vs1053b-rtmidistart.zip
// Permission to reproduce here granted by VLSI solution.
//
const unsigned short sVS1053b_Realtime_MIDI_Plugin[28] = { /* Compressed plugin */
  0x0007, 0x0001, 0x8050, 0x0006, 0x0014, 0x0030, 0x0715, 0xb080, /*    0 */
  0x3400, 0x0007, 0x9255, 0x3d00, 0x0024, 0x0030, 0x0295, 0x6890, /*    8 */
  0x3400, 0x0030, 0x0495, 0x3d00, 0x0024, 0x2908, 0x4d40, 0x0030, /*   10 */
  0x0200, 0x000a, 0x0001, 0x0050,
};

void VSLoadUserCode(void) {
  int i = 0;

  while (i<sizeof(sVS1053b_Realtime_MIDI_Plugin)/sizeof(sVS1053b_Realtime_MIDI_Plugin[0])) {
    unsigned short addr, n, val;
    addr = sVS1053b_Realtime_MIDI_Plugin[i++];
    n = sVS1053b_Realtime_MIDI_Plugin[i++];
    while (n--) {
      val = sVS1053b_Realtime_MIDI_Plugin[i++];
      VSWriteRegister(addr, val >> 8, val & 0xFF);
    }
  }
}

void VS1053_Init_SPI_MIDI(){
  
  // Initialize SPI
  pinMode(VS_DREQ, INPUT);
  pinMode(VS_XCS, OUTPUT);
  pinMode(VS_XDCS, OUTPUT);
  pinMode(VS_RESET, OUTPUT);
  digitalWrite(VS_XCS, HIGH); //Deselect Control
  digitalWrite(VS_XDCS, HIGH); //Deselect Data


  //Initialize VS1053 chip 
  digitalWrite(VS_RESET, LOW); //Put VS1053 into hardware reset

  //Setup SPI for VS1053
  pinMode(SPI_SS, OUTPUT); // SS pin must be set as an output for the SPI communication to work
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  //From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz. 
  //Internal clock multiplier is 1.0x after power up. 
  //Therefore, max SPI speed is 1.75MHz. We will use 1MHz to be safe.
  SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  SPI.transfer(0xFF); //Throw a dummy byte at the bus

  delayMicroseconds(1);
  digitalWrite(VS_RESET, HIGH); //Bring up VS1053
  VSLoadUserCode(); //Enable MIDI mode via SPI
}


// define MIDI channel messages
// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00 // default bank
#define VS1053_BANK_DRUMS1 0x78  // drums bank (unnecessary?)
#define VS1053_BANK_DRUMS2 0x7F  // drums bank (unnecessary?)
#define VS1053_BANK_MELODY 0x79  // melodic bank
#define VS1053_RPN_MSB 0x06 // most sig. bit for bend/coarse tune
#define VS1053_RPN_LSB 0x26 // leas sig. bit for bend

// define MIDI sounds
// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_SQUARE_CLICK 32 // square click
#define VS1053_GM1_CLARINET 72 // clarinet sound
#define VS1053_GM1_FLUTE 74 // flute sound
#define VS1053_GM1_WOODBLOCK 116 // woodblock sound
#define VS1053_GM1_ACOUSTICBASS 33 // acoustic bass

// more channel messages
#define MIDI_NOTE_ON  0x90     // note on
#define MIDI_NOTE_OFF 0x80     // note off
#define MIDI_CHAN_MSG 0xB0     // parameter (not sure what this does)
#define MIDI_CHAN_BANK 0x00    // calling in the default bank?
#define MIDI_CHAN_VOLUME 0x07  // channel volume
#define MIDI_CHAN_PROGRAM 0xC0 // program (not sure exactly what this does)
#define MIDI_CHAN_PITCH_WHEEL 0xE0 //pitch wheel


/*End VS1053 */

// might be old code from color arduino? maybe not functioning
// we only play a note when the clear response is higher than a certain number 
//#define CLEARTHRESHHOLD 2000 
#define CLEARTHRESHHOLD 100 // threshhold from color arduino? not sure if function
#define LOWTONE 1000
#define HIGHTONE 2000
#define LOWKEY 40 // use this - two octaves lower than high C?
//#define LOWKEY 52   // octave lower than high C? 
//#define LOWKEY 64   // high C
#define HIGHKEY 76  // double high C
// not sure the difference between 'tone' and 'key'

// defines way of looking at previous note
int prevNote = -1;

/**************************************************************************/
// variables from eclipse arduino code -- not sure what is relevant to LightSound
// some for publishing to web?
int SwitchState=0; //Variable to store state of switch to publish to web or just play audio
int SwitchStatePin=12; //Pin to publish to web (high/on), or just play audio (low/off)
int TonePin=13; //Pin to play tone output
int LedPin=0; //Pin to light LED indicator (0 on ESP8266 breakout)
int PinState=LOW; //Variable to store state of tone/led pin
unsigned long previousMicros; //stores last time tone/led pin was updated
unsigned long OnTime=100; //microseconds on-time
unsigned long OffTime=100; //microseconds off-time
unsigned long previousMillis; //stores last time light sensor was read
unsigned long ReadTime=2000; //Light sensor read interval in milliseconds
float lvl=0.0;
int noteCase=0;
int prevLvlInt=0; 
float prevLvl=0.0; 
float delayTime=0.0;
int lvlInt=0;
int delayTimeInt=0;
uint32_t x=0;
uint16_t ir=0; //infrared sensor value 16 bit
uint16_t full=0; //full spectrum sensor value 16 bit 
uint16_t vis=0;  //visible spectrum inferred from full spectrum and IR sensors
uint32_t lum=0; //32 bit full spectrum MSB's, IR LSB's

// variable to hold sensor value
int sensorValue;
// variable to calibrate low value; use bigger number for lower value to invert dependencece
int sensorLow = 0; 
// variable to calibrate high value; note A2D converter is 10 bit, so max is 1023
int sensorHigh = 32767; 
//131072, 65353 (both as unsigned longs) 32767 (int) stack errors on serial monitor; even using constrain to convert float lvl to int
int pitchLow = 1; //Note minimum pitch for tone function is 31Hz; manually write high/low to get lower pitches
int pitchHigh = 10000; //Note I can only hear up to just above 10kHz
int waitLow = 1; //wait time lower bound
int waitHigh = 16383; //wait time upper bound for delayMicroseconds()
//int sensorPin = A5; //Adafruit Circuit Playground light sensor pin (phototransistor)
int wait = 0; //wait time 
int ncycles = 10; //number of times to toggle pin
int nwait = 0; //wait factor multiplier
int nwaitLow = 0;
int nwaitHigh = 200;

/**************************************************************************/
// lux sensor setup
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345); // TSL2561 Lux sensor
//SoftwareSerial Serial2(4, VS1053_RX); //MIDI board; TX only, do not use the 'rx' side
// ESP32 board does not use SoftwareSerial. Use Serial2 instead!!!

/**************************************************************************/
// define setup loop
void setup() {
  Serial.begin(115200); // Serial Console baudrate
  Serial.println("Lux Sensor MIDI!");

  //Check for light sensor (loop with error print-out)
  // I2C Init
  I2C_BUS.begin(I2C_SDA, I2C_SCL, 100000);
  if (tsl.begin(&I2C_BUS)) {
    Serial.println("Found sensor");
  } 
  else {
    Serial.println("No TSL2561 found ... check your connections");
    //while (1); // halt!
  }

  //Start MIDI
  VS1053_Init_SPI_MIDI();
 
  //Preparing the band to play
  midiSetChannelBank(0, VS1053_BANK_MELODY); // sets melody channel for MIDI board
  midiSetInstrument(0, VS1053_GM1_CLARINET); // sets clarinet sound for MIDI instrument
  midiSetChannelVolume(0, 127);              // sets volume -- able to change volume
  player.add_instrument("/synth_1.inst");
  player.add_instrument("/synth_2.inst");
  player.add_instrument("/synth_3.inst");
  player.add_instrument("/synth_4.inst");
  player.add_drums("/percussion.inst");
}

/**************************************************************************/
/**************************************************************************/
 /****** light sensor config function definitions ******/
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
//  delay(1000); //Delay to ensure time between readouts given max 400ms integration time; AND to stay below Adafruit IO throttle rate 125 posts in 60 seconds
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  /* TSL2561 only offers 2 gain options */
  //tsl.setGain(TSL2561_GAIN_1X);    /* 1x gain (bright light) */
  tsl.setGain(TSL2561_GAIN_16X);    /* 25x gain */
  
  /* Changing the integration time gives you a longer time over which to sense light */
  /* TSL2561 only offers 3 integration time modes*/
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);   // longest integration time (dim light)

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("402 ms");
  Serial.println("------------------------------------");

/**************************************************************************/
/**************************************************************************/

  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
}

/**************************************************************************/      
// Performs a read using the Adafruit Unified Sensor API. 
/**************************************************************************/
// loop for reading with Lux sensor

void loop(){
  /* Get a new sensor event */ 
  //sendMIDI(0x50 | 0x90 | 0x26 | 0x3C);
  int valorPulsador = digitalRead(ESP32_BUTTON);
  if (valorPulsador == 1 && old_value == 0){
      Serial.println(valorPulsador);
      mode_lightsound = ++mode_lightsound % 2;
      old_value = 1;
      //sendMIDI(0xb0);
      //sendMIDI(0x7b);
      //sendMIDI(0x7c);
      //sendMIDI(0x7d);
      talkMIDI(0xb0, 0x7b, 0x00);
  }
  else if (valorPulsador == 0 && old_value == 1)
      old_value = 0;
  //advancedPlayNoteBending(0);
  int delta_trash = millis();
  lvl = advancedLightSensorRead();
  delta_trash = millis() - delta_trash;
  Serial.print("Delta Trash: ");
  Serial.println(delta_trash);
  
  /* Play the corresponding note to the lux */
  advancedPlayNoteBending(lvl, delta_trash);
}



/**************************************************************************/      
// Function defintions
/**************************************************************************/
/* definition for measurement/autogain function*/
float advancedLightSensorRead(void)
{

  // auto gain from http://microcontrolbasics.blogspot.com/2015/04/adafruit-tsl2591-lux-sensor.html
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  
  sensors_event_t event;
  tsl.getEvent(&event);

  lvl = event.light;

  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] ")); Serial.println(F("  "));
  Serial.print(F("Visible (event.light): ")); Serial.print(event.light); Serial.print(F(" Lux ")); Serial.println(F(" "));

   
  if (event.light <= 0)
  {
    tsl.setGain(TSL2561_GAIN_1X);              
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    Serial.println("Gain: 1x (Low)");
    Serial.println("Integration: 13 ms");
  }
  else if (event.light > 200.0)
  { 
    tsl.setGain(TSL2561_GAIN_1X);              
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    Serial.println("Gain: 1x (Low)");
    Serial.println("Integration: 13 ms");
  }
  else if (event.light <=200.0 && event.light > 40.0)
  {
    tsl.setGain(TSL2561_GAIN_1X);                
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
    Serial.println("Gain: 1x (Low)");
    Serial.println("Integration: 101 ms");
  }
  else if (event.light <=40.0 && event.light > 10.0)
  {
    tsl.setGain(TSL2561_GAIN_16X);                
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    Serial.println("Gain: 16x (Max)");
    Serial.println("Integration: 13 ms");
  }
  else if (event.light <=10.0 && event.light > 0.1)
  {
    tsl.setGain(TSL2561_GAIN_16X);                
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
    Serial.println("Gain: 16x (Max)");
    Serial.println("Integration: 101 ms");
  }
  else
  {
    tsl.setGain(TSL2561_GAIN_16X);                
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
    Serial.println("Gain: 16x (Max)");
    Serial.println("Integration: 402 ms");
  }
  Serial.println(" ");

  prevLvl = lvl;
  return lvl;
 
}

/* *********** */

// definition for "play note" function
void advancedPlayNoteBending(float lvl, int delta_trash) {
  if (mode_lightsound == 0){
    player.play(lvl, delta_trash);
  }
  else
     oldAdvancedPlayNoteBending(lvl);
  if (1==0){
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  File file = SPIFFS.open("/synth_1.inst");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  char dataArray[2];
  char s[2];
  byte type;
  byte note;
  byte velocity;
  int delta = 0;
  float fact = 1.04;
  Serial.println("File Content:");
  while(file.available()){
    do{
      file.readBytes(dataArray, 1);
      type = (byte)dataArray[0];
      file.readBytes(dataArray, 1);
      note = (byte)dataArray[0];
      file.readBytes(dataArray, 1);
      velocity = (byte)dataArray[0];
      file.readBytes(dataArray, 2);
      delta = ((byte)dataArray[0]) + ((byte)dataArray[1] << 8 );
      Serial.print(type);
      Serial.print(", ");
      Serial.print(note);
      Serial.print(", ");  
      Serial.print(velocity);
      Serial.print(", ");
      Serial.println(delta);
      delay((int)(delta * fact));
      midiSetChannelBank(0, VS1053_BANK_MELODY);
      midiSetInstrument(0, 27);
      if (type == 1)
          midiNoteOn(0, note, velocity);
      else
          midiNoteOff(0, note, velocity);
    }while(delta == 0);
  }
  file.close();
  }


  //midiSetChannelBank(0, VS1053_BANK_DRUMS2);
  //midiSetInstrument(0, 38);
  
  //midiNoteOn(0, percussion_array[ind_global], percussion_velocity_array[ind_global]);




  //#midiNoteOn(0, drums[ind_drum], 127);
    


  //if (lvl > -1){
  //midiSetChannelBank(0, VS1053_BANK_MELODY);
  //midiSetInstrument(0, 27);
  //if (synth_2_array[ind_global] != 0)
  //midiNoteOn(0, percussion_array[ind_global], percussion_velocity_array[ind_global]);
  //midiSetInstrument(0, 55);
  //if (array2[ind_global] != 0)
  //  midiNoteOn(0, array2[ind_global], 50);
  //}

  //delay(10);
  //midiNoteOff(0, percussion_array[ind_global], 0);
  //delay(percussion_time_array[ind_global + 1]);
  //midiNoteOff(0, percussion_array[ind_global], 0);
  //ind_global++;
}



// definition for float map function 
// from: https://forum.arduino.cc/index.php?topic=3922.0 (post #2)
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 

/*
 * micros() overflows in ~70 minutes; 4us resolution
 * millis() overflows in ~50 days
 * 
 * light level ranges from 1/24th to 65,536*2=131072; Range is 131072*24=3,145,728
 * sound level ranges from 0.1 Hz to 10kHz; Range is ~10^5
 * 
 * avoiding fractions requires dealing with periods rather than frequencies
 * resolution limited to integer changes in milliseconds (1ms) or 4us micros() resolution
 * 0.1Hz-10kHz -> 100ms-100usec
 * weight lower light level range; compress upper light level range
 */
/**************************/

void sendMIDI(byte data)
{
  SPI.transfer(0);
  SPI.transfer(data);
}

//Plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that data values are less than 127
void talkMIDI(byte cmd, byte data1, byte data2) {
  //
  // Wait for chip to be ready (Unlikely to be an issue with real time MIDI)
  //
  while (!digitalRead(VS_DREQ))
    ;
  digitalWrite(VS_XDCS, LOW);
  sendMIDI(cmd);
  //Some commands only have one data byte. All cmds less than 0xBn have 2 data bytes 
  //(sort of: http://253.ccarh.org/handout/midiprotocol/)
  if( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) >= 0xE0) {
    sendMIDI(data1);
    sendMIDI(data2);
  } else {
    sendMIDI(data1);
  }

  digitalWrite(VS_XDCS, HIGH);
}

//Send a MIDI note-on message.  Like pressing a piano key
//channel ranges from 0-15
void midiNoteOn(byte channel, byte note, byte attack_velocity) {
  if (channel > 15) return;
  if (note > 127) return;
  if (attack_velocity > 127) return;
  talkMIDI( (0x90 | channel), note, attack_velocity);
}

//Send a MIDI note-off message.  Like releasing a piano key
void midiNoteOff(byte channel, byte note, byte release_velocity) {
  if (channel > 15) return;
  if (note > 127) return;
  if (release_velocity > 127) return;
  talkMIDI( (0x80 | channel), note, release_velocity);
}

void midiSetInstrument(byte chan, byte inst) {
  inst --; // page 32 has instruments starting with 1 not 0 :(
  talkMIDI(MIDI_CHAN_PROGRAM | chan, inst, 0);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;

  sendMIDI(MIDI_CHAN_MSG | chan);
  sendMIDI(MIDI_CHAN_VOLUME);
  sendMIDI(vol);
}

void midiSetChannelBank(byte chan, byte bank) {

  //sendMIDI((uint8_t)MIDI_CHAN_BANK);
  //sendMIDI();
  talkMIDI(MIDI_CHAN_MSG | chan, 0, bank); 

}

// definition for MIDI "note bend" function

/* from: https://arduino.stackexchange.com/questions/18955/how-to-send-a-pitch-bend-midi-message-using-arcore
   The pitch bend value is a 14-bit number (0-16383). 0x2000 (8192) is the default / middle value.
   First byte is the event type (0x0E = pitch bend change).
   Second byte is the event type, combined with the channel.
   Third byte is the 7 least significant bits of the value.
   Fourth byte is the 7 most significant bits of the value. */

void pitchBendChange(byte channel, int value) {
  byte lowValue = value & 0x7F;
  byte highValue = value >> 7;
  sendMIDI(0xE0 | channel);
  sendMIDI(lowValue);
  sendMIDI(highValue);
}

void oldAdvancedPlayNoteBending(float lvl) {
  if (lvl < 61 & lvl >= 0){
    //stop last note played
    if (noteCase == 1) {
      midiNoteOff(0, prevNote, 100);
    }

    else if (noteCase == 2) {
      midiNoteOff(0, prevNote, 62);
    }

    lvlInt = round(lvl);
    delayTime = 1000/lvl;
    delayTimeInt = round(delayTime);
    
    //talkMIDI(0xB0, 0, VS1053_BANK_DRUMS1); //Bank select: drums
    //talkMIDI(0xC0, VS1053_GM1_SQUARE_CLICK, 0); //Set instrument number
    midiSetChannelBank(0, VS1053_BANK_DRUMS1);
    midiSetInstrument(0, VS1053_GM1_SQUARE_CLICK);

    if (lvlInt > 0) {
      for (int count = 0; count < lvlInt; count++) {
        midiNoteOn(0, 69, 127);
        delay(5);
        midiNoteOff(0, 69, 127);
        delay(delayTimeInt);
      }
    }

    else if (lvlInt == 0 & lvl > 0){
      midiNoteOn(0, 69, 127);
      delay(5);
      midiNoteOff(0, 69, 127);
      delay(2000);
    }
   
   noteCase = 0;
   prevLvlInt = lvlInt;
  }

  else if (lvl >= 61) {
    if (lvl <= 10000) {
      float nFloat = mapfloat(lvl,61,10000,35,81);
      
      //extract decimal
      int nInt = nFloat;
      String stringval = String(nFloat);
      String mantissa = stringval.substring(stringval.lastIndexOf(".") + 1, stringval.lastIndexOf(".") + 2);
      int decimalInt = mantissa.toInt();
      int decMap = map(decimalInt,0,10,8192,16383);        

      if (nInt == prevNote & noteCase == 1) return;
    
      if (nInt == -1 & noteCase == 1){
        midiNoteOff(0, prevNote, 100);
      }

      else if (nInt == -1 & noteCase == 2){
        midiNoteOff(0, prevNote, 62);
      }
    
      //stop last note played
      if (noteCase == 1){
        midiNoteOff(0, prevNote, 100);
      }

      else if (noteCase == 2){
        midiNoteOff(0, prevNote, 62);
      }

      //play new note with bend
      //talkMIDI(0xB0, 0, VS1053_BANK_MELODY);
      //talkMIDI(0xC0, VS1053_GM1_CLARINET, 0);
      midiSetChannelBank(0, VS1053_BANK_MELODY);
      midiSetInstrument(0, VS1053_GM1_CLARINET);
      midiNoteOn(0, nInt, 100);
      pitchBendChange(0,decMap);     

      prevNote = nInt;
      noteCase = 1;
    }
    
    else {   
      float nHighFloat = mapfloat(lvl,10000,131072,81,90);
      
      //extract decimal
      int nHighInt = nHighFloat;
      String stringvalHigh = String(nHighFloat);
      String mantissaHigh = stringvalHigh.substring(stringvalHigh.lastIndexOf(".") + 1, stringvalHigh.lastIndexOf(".") + 2);
      int decimalIntHigh = mantissaHigh.toInt();
      int decMapHigh = map(decimalIntHigh,0,10,8192,16383);

      if (nHighInt == prevNote & noteCase == 2) return;
   
      if (nHighInt == -1 & noteCase == 1){
        midiNoteOff(0, prevNote, 100);
      }

      else if (nHighInt == -1 & noteCase == 2){
        midiNoteOff(0, prevNote, 62);
      }

      if (noteCase == 1){
        midiNoteOff(0, prevNote, 100);
      }

      else if (noteCase == 2){
        midiNoteOff(0, prevNote, 62);
      }
      
      midiSetChannelBank(0, VS1053_BANK_MELODY);
      midiSetInstrument(0, VS1053_GM1_FLUTE);
      //talkMIDI(0xB0, 0, VS1053_BANK_MELODY);
      //talkMIDI(0xC0, VS1053_GM1_FLUTE, 0);
      midiNoteOn(0, nHighInt, 62);
      pitchBendChange(0,decMapHigh);
      delay(500);

      prevNote = nHighInt;
      noteCase = 2;
    } 
  }

  else if (lvl < -5){
    if (noteCase == 1){
        midiNoteOff(0, prevNote, 100);
      }

    if (noteCase == 2){
        midiNoteOff(0, prevNote, 62);
      }

    //midiSetChannelBank(0, VS1053_BANK_MELODY);
    //midiSetInstrument(0, VS1053_GM1_ACOUSTICBASS);
    talkMIDI(0xB0, 0, VS1053_BANK_MELODY);
    talkMIDI(0xC0, VS1053_GM1_ACOUSTICBASS, 0);

    midiNoteOn(0, 90, 88);
    pitchBendChange(0,8192);
    delay(500);
    midiNoteOff(0, 90, 88);
    delay(1000);
    noteCase = 3;
    
  }
}
