/*
  Player.h - Library for play instruments.
  Created  by Carlos S., December 1, 2019.
  Released into the public domain.
*/

#ifndef Player_h
#define Player_h

#include "Instrument.h"
#include "Arduino.h"
#include "SPIFFS.h"
#include <chrono>

class Player
{
    public:
        Player();
        void add_instrument(char* file_name);
        void add_drums(char* file_name);
        void play(float lvl, int delta_trash);

    private:
        float _fact;
        int _delay_time;
        bool _frozen;
        uint32_t _time_total;
        uint8_t _total_inst;
        uint8_t _total_drums;
        Instrument instruments[15];
        Instrument drums[15];
        void _heating(int delta_trash);
        float _mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif
