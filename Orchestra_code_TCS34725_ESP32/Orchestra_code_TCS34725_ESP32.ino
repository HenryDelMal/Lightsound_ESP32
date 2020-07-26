#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>
#include "Flora_Pianoglove.h"
#include <SPI.h>

#define I2C_SDA 13
#define I2C_SCL 15

#define NEOPIXEL_PIN 14

#define VS_XCS    26 // Control Chip Select Pin (for accessing SPI Control/Status registers)
#define VS_XDCS   33 // Data Chip Select / BSYNC Pin
#define VS_DREQ   32 // Data Request Pin: Player asks for more data
#define SPI_SS    5 // Pin for enable SPI on some boards

#define VS_RESET  25 //Reset is active low

// I2C Init
TwoWire I2C_BUS = TwoWire(0);

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

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_ACOUSTICGRANDPIANO 1
#define VS1053_GM1_BRIGHTACOUSTICPIANO 2
#define VS1053_GM1_ACCORDION 22
#define VS1053_GM1_ACOUSTICGUITARNYLON 25
#define VS1053_GM1_ACOUSTICGUITARSTEEL 26
#define VS1053_GM1_ELECTRICGUITARCLEAN 28
#define VS1053_GM1_ACOUSTICBASS 33
#define VS1053_GM1_ELECTRICBASSPICK 35
#define VS1053_GM1_CELLO 43
#define VS1053_GM1_TUBA 59
#define VS1053_GM1_BASSOON 71
#define VS1053_GM1_CLARINET 72
#define VS1053_GM1_OCARINA 80
#define VS1053_GM1_SQUARELEAD 81
#define VS1053_GM1_SAWLEAD 82
#define VS1053_GM1_CALLIOPELEAD 83
#define VS1053_GM1_CHIFFLEAD 84
#define VS1053_GM1_CHARANGLEAD 85
#define VS1053_GM1_VOICELEAD 86
#define VS1053_GM1_FIFTHSLEAD 87
#define VS1053_GM1_BASSANDLEAD 88
#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

/*End VS1053 */


// we only play a note when the clear response is higher than a certain number 
//#define CLEARTHRESHHOLD 2000 
#define CLEARTHRESHHOLD 100 
#define LOWTONE 1000
#define HIGHTONE 2000
#define LOWKEY 40 // use this - two octaves lower than high C?
//#define LOWKEY 52   // octave lower than high C? 
//#define LOWKEY 64   // high C
#define HIGHKEY 76  // double high C


// our RGB -> eye-recognized gamma color
byte gammatable[256];

int prevNote = -1;

// color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// one pixel on pin 6 (change to pin 8 on Flora v3) - revert to pin 6 when using added chassis-mounted neopixel
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);


void setup() {
  
  Serial.begin(115200);
  Serial.println("Piano Glove MIDI!");

  // I2C Init
  I2C_BUS.begin(I2C_SDA, I2C_SCL, 100000);
  //Check for color sensor
  if (tcs.begin(0x29,&I2C_BUS)) {
    Serial.println("Found sensor");
  } 
  else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  //init neopixel for color 'playback'
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  //Start MIDI
  Serial.begin(115200);
  Serial.println("VS1053 MIDI test");

  VS1053_Init_SPI_MIDI();
  //midiSetChannelBank(0, VS1053_BANK_MELODY);
  //midiSetInstrument(0, VS1053_GM1_BASSANDLEAD); //wierd
  //midiSetInstrument(0, VS1053_GM1_ACOUSTICGRANDPIANO); //OK but try other pianos
  midiSetInstrument(0, VS1053_GM1_BRIGHTACOUSTICPIANO); //Acoustic Grand Piano (#1) better; too much reverb
  //midiSetInstrument(0, VS1053_GM1_ACCORDION); //Sounds like the Adams Family theme song
  //midiSetInstrument(0, VS1053_GM1_ELECTRICGUITARCLEAN); //OK; not great
  //midiSetInstrument(0, VS1053_GM1_ACOUSTICGUITARNYLON);
  //midiSetInstrument(0, VS1053_GM1_ACOUSTICGUITARSTEEL); //Second best so far
  //midiSetInstrument(0, VS1053_GM1_ACOUSTICBASS); //Best so far
  //midiSetInstrument(0, VS1053_GM1_CLARINET); // Soley prefers- sustains
  //midiSetInstrument(0, VS1053_GM1_ELECTRICBASSPICK); //Acoustic Bass is cleaner
  //midiSetInstrument(0, VS1053_GM1_CELLO); //Reverb to harsh, long - sounds like a horn
  //midiSetInstrument(0, VS1053_GM1_TUBA); //Smoother than Cello but reverb still somewhat harsh
  //midiSetInstrument(0, VS1053_GM1_BASSOON); //Annoying!
  //midiSetInstrument(0, 16); //Dulcimer - No; Annoying as well.
  
  midiSetChannelVolume(0, 127);

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    gammatable[i] = x;      

  }
}


void loop() {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 

  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED

  // not close enough to colorful item
  if (clear < CLEARTHRESHHOLD) {
    playNote(-1);
    strip.setPixelColor(0, strip.Color(0, 0, 0)); // turn off the LED
    strip.show();
    return;
  }

    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.print(blue);
    Serial.println();

  // Figure out some basic hex code for visualization
  uint32_t sum = red;
  sum += green;
  sum += blue;
  sum = clear;
  float r, g, b;
  r = red; 
  r /= sum;
  g = green; 
  g /= sum;
  b = blue; 
  b /= sum;
  r *= 256; 
  g *= 256; 
  b *= 256;
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;

  //  Serial.print("\t");
  //  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX); 
  //  Serial.println();


  // OK we have to find the two primary colors
  // check if blue is smallest. MEME: fix for 'white'
  float remove, normalize;
  if ((b < g) && (b < r)) {
    remove = b;
    normalize = max(r-b, g-b);
  } 
  else if ((g < b) && (g < r)) {
    remove = g;
    normalize = max(r-g, b-g);
  } 
  else {
    remove = r;
    normalize = max(b-r, g-r);
  }
  // get rid of minority report
  float rednorm = r - remove;
  float greennorm = g - remove;
  float bluenorm = b - remove;
  // now normalize for the highest number
  rednorm /= normalize;
  greennorm /= normalize;
  bluenorm /= normalize;

  //  Serial.println();
  strip.setPixelColor(0, strip.Color(gammatable[(int)r], gammatable[(int)g], gammatable[(int)b]));
  strip.show();

  //  Serial.print(rednorm); Serial.print(", "); 
  //  Serial.print(greennorm); Serial.print(", "); 
  //  Serial.print(bluenorm); Serial.print(" "); 
  //  Serial.println();

  float rainbowtone = 0;

  if (bluenorm <= 0.1) {
    // between red and green
    if (rednorm >= 0.99) {
      // between red and yellow
      rainbowtone = 0 + 0.2 * greennorm;
    } 
    else {
      // between yellow and green
      rainbowtone = 0.2 + 0.2 * (1.0 - rednorm);
    }
  } 
  else if (rednorm <= 0.1) {
    // between green and blue
    if (greennorm >= 0.99) {
      // between green and teal
      rainbowtone = 0.4 + 0.2 * bluenorm;
    } 
    else {
      // between teal and blue
      rainbowtone = 0.6 + 0.2 * (1.0 - greennorm);
    }
  } 
  else {
    // between blue and violet
    if (bluenorm >= 0.99) {
      // between blue and violet
      rainbowtone = 0.8 + 0.2 * rednorm;
    } 
    else {
      // between teal and blue
      rainbowtone = 0; 
    }
  }

    Serial.print("Scalar "); Serial.println(rainbowtone);
  float keynum = LOWKEY + (HIGHKEY - LOWKEY) * rainbowtone;
    Serial.print("Key #"); Serial.println(keynum);
    float freq = pow(2, (keynum - 49) / 12.0) * 440;
    Serial.print("Freq = "); Serial.println(freq); 
    Serial.println(" "); 
  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
  playNote(keynum);
}


void playNote(uint16_t n) {
  
  if (n == prevNote) return;
  
  if (n == -1){
    midiNoteOff(0, prevNote, 127);
  }
  
  //stop last note played
  midiNoteOff(0, prevNote, 127);
  
  //play new note
  midiNoteOn(0, n, 127);
  
  prevNote = n;
}


RgbColor HsvToRgb(HsvColor hsv){
  
  RgbColor rgb;
  unsigned char region, remainder, p, q, t;

  if (hsv.s == 0)
  {
    rgb.r = hsv.v;
    rgb.g = hsv.v;
    rgb.b = hsv.v;
    return rgb;
  }

  region = hsv.h / 43;
  remainder = (hsv.h - (region * 43)) * 6; 

  p = (hsv.v * (255 - hsv.s)) >> 8;
  q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
  t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

  switch (region)
  {
  case 0:
    rgb.r = hsv.v; 
    rgb.g = t; 
    rgb.b = p;
    break;
  case 1:
    rgb.r = q; 
    rgb.g = hsv.v; 
    rgb.b = p;
    break;
  case 2:
    rgb.r = p; 
    rgb.g = hsv.v; 
    rgb.b = t;
    break;
  case 3:
    rgb.r = p; 
    rgb.g = q; 
    rgb.b = hsv.v;
    break;
  case 4:
    rgb.r = t; 
    rgb.g = p; 
    rgb.b = hsv.v;
    break;
  default:
    rgb.r = hsv.v; 
    rgb.g = p; 
    rgb.b = q;
    break;
  }

  return rgb;
}


HsvColor RgbToHsv(RgbColor rgb){
  
  HsvColor hsv;
  unsigned char rgbMin, rgbMax;

  rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : 
  (rgb.g < rgb.b ? rgb.g : rgb.b);
  rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : 
  (rgb.g > rgb.b ? rgb.g : rgb.b);

  hsv.v = rgbMax;
  if (hsv.v == 0)
  {
    hsv.h = 0;
    hsv.s = 0;
    return hsv;
  }

  hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
  if (hsv.s == 0)
  {
    hsv.h = 0;
    return hsv;
  }

  if (rgbMax == rgb.r)
    hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
  else if (rgbMax == rgb.g)
    hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
  else
    hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

  return hsv;
}


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

void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;

  sendMIDI(MIDI_CHAN_PROGRAM | chan);
  sendMIDI(inst);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;

  sendMIDI(MIDI_CHAN_MSG | chan);
  sendMIDI(MIDI_CHAN_VOLUME);
  sendMIDI(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;

  sendMIDI(MIDI_CHAN_MSG | chan);
  sendMIDI((uint8_t)MIDI_CHAN_BANK);
  sendMIDI(bank);
}
