/***************************************************
  This is an example for the Adafruit VS1053 Codec Breakout
  Designed specifically to work with the Adafruit VS1053 Codec Breakout
  ----> https://www.adafruit.com/products/1381
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// define the pins used
#define VS1053_RX  17 // This is the pin that connects to the RX pin on VS1053

#define VS1053_RESET 19 // This is the pin that connects to the RESET pin on VS1053
// If you have the Music Maker shield, you don't need to connect the RESET pin!

// If you're using the VS1053 breakout:
// Don't forget to connect the GPIO #0 to GROUND and GPIO #1 pin to 3.3V
// If you're using the Music Maker shield:
// Don't forget to connect the GPIO #1 pin to 3.3V and the RX pin to digital #2

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

void setup() {
  Serial.begin(115200);
  Serial.println("VS1053 MIDI test");

  Serial2.begin(31250, SERIAL_8N1, 16, 17); // MIDI uses a 'strange baud rate'

  pinMode(VS1053_RESET, OUTPUT);
  digitalWrite(VS1053_RESET, LOW);
  delay(10);
  digitalWrite(VS1053_RESET, HIGH);
  delay(10);

  midiSetChannelBank(0, VS1053_BANK_DRUMS2);
  midiSetInstrument(0, VS1053_GM1_OCARINA);
  midiSetChannelVolume(0, 127);
}

void loop() {
  for (uint8_t i = 60; i < 69; i++) {
    midiNoteOn(0, i, 127);
    delay(100);
    Serial.println(i);
    midiNoteOff(0, i, 127);
  }

  delay(1000);
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

  Serial2.write(MIDI_NOTE_ON | chan);
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
