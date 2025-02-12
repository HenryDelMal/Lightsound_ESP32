#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>
#include "Flora_Pianoglove.h"
#include "VS1053.h"
#include "SPIFFS.h"

int mode_lightsound = 0;
int old_value = 0;

// define the pins used
#define VS1053_RXpo17 // This is the pin that connects to the RX pin on VS1053
#define VS1053_RESET 15 // This is the pin that connects to the RESET pin on VS1053
#define NEOPIXEL_PIN 25 // This is the pin that connects to the Neopixel.
// Don't forget to connect the GPIO #0 to GROUND and GPIO #1 pin to 3.3V
#define VOLUME  100 // volume level 0-100
#define VS1053_CS     5
#define VS1053_DCS    16
#define VS1053_DREQ   4
#define ESP32_BUTTON 32
#define ESP32_MIDI 26


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

VS1053 player(VS1053_CS, VS1053_DCS, VS1053_DREQ);
char* amarillo;
char* azul;
char* blanco;
char* cafe;
char* cian;
char* gris;
char* magenta;
char* naranja; 
char* negro;
char* rojo;
char* verde;
char* violeta;

int amarillo_size, azul_size, blanco_size, cafe_size, cian_size, gris_size;
int magenta_size, naranja_size, negro_size, rojo_size, verde_size, violeta_size;


void setup() {
  
  pinMode(ESP32_MIDI, OUTPUT);
  pinMode(ESP32_BUTTON, INPUT);
  
  Serial.begin(9600);
  Serial.println("Piano Glove MIDI!");

  //Check for color sensor
  if (tcs.begin()) {
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
  Serial2.begin(31250); // MIDI uses a 'strange baud rate'
  digitalWrite(ESP32_MIDI, HIGH);
  pinMode(VS1053_RESET, OUTPUT);
  digitalWrite(VS1053_RESET, LOW);
  delay(10);
  digitalWrite(VS1053_RESET, HIGH);
  delay(10);
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
  amarillo_size = read_file(amarillo, "/Amarillo.mp3");
  azul_size = read_file(azul, "/Azul.mp3");
  blanco_size = read_file(blanco, "/Blanco.mp3");
  cafe_size = read_file(cafe, "/Café.mp3");
  azul_size = read_file(azul, "/Azul.mp3");
  cian_size = read_file(cian, "/Cian.mp3");
  gris_size = read_file(gris, "/Gris.mp3");
  magenta_size = read_file(magenta, "/Magenta.mp3");
  naranja_size = read_file(naranja, "/Naranja.mp3");
  negro_size = read_file(negro, "/Negro.mp3");
  rojo_size = read_file(rojo, "/Rojo.mp3");
  verde_size = read_file(verde, "/Verde.mp3");
  violeta_size = read_file(violeta, "/Violeta.mp3");

}


void loop() {
  int valorPulsador = digitalRead(ESP32_BUTTON);

  if (valorPulsador == 1 && old_value == 0){
      Serial.println(valorPulsador);
      mode_lightsound = ++mode_lightsound % 2;
      old_value = 1;
      Serial2.write(0xb0);
      Serial2.write(0x7b);
      Serial2.write(0x7c);
      Serial2.write(0x7d);
      if (mode_lightsound == 1){
        Serial.println("To mp32");
        delay(2000);
        digitalWrite(ESP32_MIDI, LOW);
        digitalWrite(VS1053_RESET, LOW);
        delay(10);
        digitalWrite(VS1053_RESET, HIGH);
        delay(10);
        initMP3Player();
      }
      else{
        Serial.println("To Sensor");
        delay(2000);
        player.softReset();
        SPI.end();
        Serial2.begin(31250); // MIDI uses a 'strange baud rate'
        digitalWrite(ESP32_MIDI, HIGH);
        digitalWrite(VS1053_RESET, LOW);
        delay(10);
        digitalWrite(VS1053_RESET, HIGH);
        delay(10);
      }
  }
  else if (valorPulsador == 0 && old_value == 1)
      old_value = 0;

  readColor();

}

int read_file(char * &color, char* colorName){
  int ret = 0;
  Serial.println(colorName);
  
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return 0;
  }
  File file = SPIFFS.open(colorName);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return 0;
  }

  if (file.available()) {
    color = (char *)malloc(sizeof(*color) * file.size());
    Serial.println(file.size());

    file.readBytes(color, file.size());
    ret = file.size();
   
  }
  file.close();

  return ret;

}

void initMIDI(){
  
}

void initMP3Player(){

    Serial.begin(115200);
    // initialize SPI
    SPI.begin();

    Serial.println("Hello VS1053!\n");
    // initialize a player
    player.begin();
    player.switchToMp3Mode(); // optional, some boards require this
    player.setVolume(VOLUME);
 

   /*    
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  File file = SPIFFS.open("/Blanco.mp3");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  
  if (file.available()) {
    char dataArray[file.size()];
    file.readBytes(dataArray, file.size());
    player.playChunk((unsigned char *)dataArray, sizeof(dataArray));
    
   
  }
  file.close();
  */
}


void readColor(){
  
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

  Serial.print(rednorm); Serial.print(", "); 
  Serial.print(greennorm); Serial.print(", "); 
  Serial.print(bluenorm); Serial.print(" "); 
  Serial.println();

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
  if(mode_lightsound == 0) 
      playNote(keynum);
  else
      readColor(freq);

}

void readColor(float freq) {
  if (freq >= 1300)
    player.playChunk((unsigned char *)violeta, violeta_size);
  else if (freq >= 1100)
    player.playChunk((unsigned char *)azul, azul_size);
  else if (freq >= 900)
    player.playChunk((unsigned char *)cian, cian_size);
  else if (freq >= 850)
    player.playChunk((unsigned char *)negro, negro_size);
  else if (freq >= 820)
    player.playChunk((unsigned char *)blanco, blanco_size);
  else if (freq >= 750)
    player.playChunk((unsigned char *)gris, gris_size);
  else if (freq >= 650)
    player.playChunk((unsigned char *)verde, verde_size);
  else if (freq >= 370)
    player.playChunk((unsigned char *)amarillo, amarillo_size);
  else if (freq >= 280)
    player.playChunk((unsigned char *)naranja, naranja_size);
  else if (freq >= 270)
    player.playChunk((unsigned char *)cafe, cafe_size);
  else if (freq >= 260)
    player.playChunk((unsigned char *)rojo, rojo_size);
     

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


void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  Serial2.write(MIDI_CHAN_PROGRAM | chan);  
  Serial2.write(inst);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  Serial2.write(MIDI_CHAN_MSG | chan);
  Serial2.write(MIDI_CHAN_VOLUME);
  Serial2.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  Serial2.write(MIDI_CHAN_MSG | chan);
  Serial2.write((uint8_t)MIDI_CHAN_BANK);
  Serial2.write(bank);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  Serial2.write(MIDI_NOTE_ON);
  Serial2.write(n);
  Serial2.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  Serial2.write(MIDI_NOTE_OFF | chan);
  Serial2.write(n);
  Serial2.write(vel);
}
