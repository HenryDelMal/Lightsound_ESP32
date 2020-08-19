/*
  Instrument.h
  Created  by Carlos S., December 1, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Instrument.h"
#include "SPIFFS.h"


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

  Serial2.write(0xC0 | chan);
  Serial2.write(inst);
}

// definition for MDID volume function
void Instrument::amidiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;

  Serial2.write(0xB0 | chan);
  Serial2.write(0x07);
  Serial2.write(vol);
}

// definition for MIDI bank function (i.e. default v.s. drums v.s. melodic?)
void Instrument::amidiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;

  Serial2.write(0xB0 | chan);
  Serial2.write((uint8_t)0x00);
  Serial2.write(bank);
}

// definition for MIDI "note on" function
void Instrument::amidiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;

  Serial2.write(0x90 | chan);
  Serial2.write(n);
  Serial2.write(vel);
}

// definition for MIDI "note off" function
void Instrument::amidiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;

  Serial2.write(0x80 | chan);
  Serial2.write(n);
  Serial2.write(vel);
}
