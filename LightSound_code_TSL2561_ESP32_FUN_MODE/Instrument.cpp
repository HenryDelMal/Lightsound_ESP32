/*
  Instrument.h
  Created  by Carlos S., December 1, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Instrument.h"
#include "SPIFFS.h"
#include <SPI.h>

#define VS_XCS    26 // Control Chip Select Pin (for accessing SPI Control/Status registers)
#define VS_XDCS   33 // Data Chip Select / BSYNC Pin
#define VS_DREQ   32 // Data Request Pin: Player asks for more data
#define SPI_SS    5 // Pin for enable SPI on some boards

#define VS_RESET  25 //Reset is active low

// more channel messages
#define MIDI_NOTE_ON  0x90     // note on
#define MIDI_NOTE_OFF 0x80     // note off
#define MIDI_CHAN_MSG 0xB0     // parameter (not sure what this does)
#define MIDI_CHAN_BANK 0x00    // calling in the default bank?
#define MIDI_CHAN_VOLUME 0x07  // channel volume
#define MIDI_CHAN_PROGRAM 0xC0 // program (not sure exactly what this does)
#define MIDI_CHAN_PITCH_WHEEL 0xE0 //pitch wheel


Instrument::Instrument()
{

}

Instrument::Instrument(char* file_name, uint8_t type, int id)
{
  _id = id;
  time_current = 0;
  _type_instrument = type;
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  file = SPIFFS.open(file_name);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  if (file.available()) {
    char dataArray[2];
    file.readBytes(dataArray, 1);
    _channel = (byte)dataArray[0];

    file.readBytes(dataArray, 1);
    _instrument = (byte)dataArray[0];
    Serial.print("Channel: ");
    Serial.print(_channel);
    Serial.print(", Id: ");
    Serial.print(_id);
    Serial.print(", Instrument: ");
    Serial.println(_instrument);
  }

  next_note();
}

TNVD Instrument::next_note()
{
  if (file.available()) {
    char dataArray[2];

    file.readBytes(dataArray, 1);
    uint8_t type = (byte)dataArray[0];

    file.readBytes(dataArray, 1);
    uint8_t note = (byte)dataArray[0];

    file.readBytes(dataArray, 1);
    uint8_t velocity = (byte)dataArray[0];

    file.readBytes(dataArray, 2);
    uint32_t delta = ((byte)dataArray[0]) + ((byte)dataArray[1] << 8 );

    current = {type, note, velocity, delta};
    time_current += current.delta;
  }
  else {
    //Serial.print("Bye ");
    //Serial.println(_id);
    file.close();
    current = { -1, -1, -1, -1};
  }
  //Serial.print("En ");
  //Serial.print(_id);
  //Serial.print(" tiempo actual: ");
  //Serial.println(time_current);
  return current;

}

void Instrument::play(uint32_t time_total, float lvl)
{
  int del_vel = lvl;
  //Serial.print("En ");
  //Serial.print(_id);
  //Serial.print(" para el tiempo total: ");
  //Serial.print(time);
  //Serial.print(" se tiene tiempo actual: ");
  //Serial.println(time_current);
  if (time_total == time_current) {
    do {
      if (_type_instrument == 0) {
        amidiSetChannelBank(_channel, 0x79);
      }
      else
        amidiSetChannelBank(_channel, 0x78);

      amidiSetInstrument(_channel, _instrument);
      int vel = current.velocity - del_vel;
      vel = vel > 0 ? vel : 0;
      if (current.type == 1)
        amidiNoteOn(_channel, current.note, vel);
      else
        amidiNoteOff(_channel, current.note, vel);
    } while (next_note().delta == 0);
  }
}


// definition for MIDI instrument function
void Instrument::amidiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;

  sendMIDI(MIDI_CHAN_PROGRAM | chan);
  sendMIDI(inst);
}

// definition for MDID volume function
void Instrument::amidiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;

  sendMIDI(MIDI_CHAN_MSG | chan);
  sendMIDI(MIDI_CHAN_VOLUME);
  sendMIDI(vol);
}

// definition for MIDI bank function (i.e. default v.s. drums v.s. melodic?)
void Instrument::amidiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;

  sendMIDI(MIDI_CHAN_MSG | chan);
  sendMIDI((uint8_t)MIDI_CHAN_BANK);
  sendMIDI(bank);
}

// definition for MIDI "note on" function
void Instrument::amidiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  talkMIDI( (0x90 | chan), n, vel);
}

// definition for MIDI "note off" function
void Instrument::amidiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  talkMIDI( (0x80 | chan), n, vel);
}

void Instrument::sendMIDI(byte data)
{
  SPI.transfer(0);
  SPI.transfer(data);
}

//Plays a MIDI note. Doesn't check to see that cmd is greater than 127, or that data values are less than 127
void Instrument::talkMIDI(byte cmd, byte data1, byte data2) {
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
