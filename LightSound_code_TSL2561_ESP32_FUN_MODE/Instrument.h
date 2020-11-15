/*
  Instrument.h
  Created  by Carlos S., December 1, 2019.
  Released into the public domain.
*/

#ifndef Instrument_h
#define Instrument_h

#include "Arduino.h"
#include "SPIFFS.h"
#include <SPI.h>



typedef struct TNVD
{
    uint8_t type, note, velocity;
    uint32_t delta;
} NVD;

class Instrument
{
    public:
        uint32_t time_current; 
        bool finish;
        TNVD current;
        Instrument();
        Instrument(char* file_name, uint8_t type, int id);
        TNVD next_note();
        void play(uint32_t time_total, float lvl);
        void amidiSetInstrument(uint8_t chan, uint8_t inst);
        void amidiSetChannelVolume(uint8_t chan, uint8_t vol);
        void amidiSetChannelBank(uint8_t chan, uint8_t bank);
        void amidiNoteOn(uint8_t chan, uint8_t n, uint8_t vel);
        void amidiNoteOff(uint8_t chan, uint8_t n, uint8_t vel);
        void talkMIDI(byte cmd, byte data1, byte data2);
        void sendMIDI(byte data);

    private:
        uint8_t _channel;
        uint8_t _instrument;
        uint8_t _id;
        uint8_t _type_instrument;
        File file;
};

#endif
