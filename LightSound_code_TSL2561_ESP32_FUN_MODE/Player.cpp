/*
  Player.cpp - Library for play instruments.
  Created  by Carlos S., December 1, 2019.
  Released into the public domain.
*/

#include "Player.h"
#include "Arduino.h"
#include "Instrument.h"
#include "SPIFFS.h"


Player::Player()
{
    _fact = 0.8;//0.65; //0.7; //1.04
    _time_total = 0;
    _total_inst = 0;
    _frozen = false;
    _delay_time = 0;

}

void Player::add_instrument(char* file_name)
{
    uint8_t id = _total_inst;
    instruments[_total_inst++] = Instrument(file_name, 0, id);

}

void Player::add_drums(char* file_name)
{
    uint8_t id = _total_drums + 15;
    drums[_total_drums++] = Instrument(file_name, 1, id);
}


void Player::play(float lvl, int delta_trash)
{
  if (lvl < 61 & lvl >= 0){
      int lvlInt = round(lvl);
  }
  else if (lvl <= 10000) {
      float nFloat = _mapfloat(lvl,61,10000,35,81);
      
      //extract decimal
      int nInt = nFloat;
      String stringval = String(nFloat);
      String mantissa = stringval.substring(stringval.lastIndexOf(".") + 1, stringval.lastIndexOf(".") + 2);
      int decimalInt = mantissa.toInt();
      int decMap = map(decimalInt,0,10,8192,16383);
    }
  else {   
      float nHighFloat = _mapfloat(lvl,10000,131072,81,90);
      
      //extract decimal
      int nHighInt = nHighFloat;
      String stringvalHigh = String(nHighFloat);
      String mantissaHigh = stringvalHigh.substring(stringvalHigh.lastIndexOf(".") + 1, stringvalHigh.lastIndexOf(".") + 2);
      int decimalIntHigh = mantissaHigh.toInt();
      int decMapHigh = map(decimalIntHigh,0,10,8192,16383);
    }

    Serial.println(lvl);
    if(!_frozen){
      lvl = 13107 - lvl > 0 ? 65536 - lvl: 0;
      lvl = _mapfloat(lvl,61,65536,0,127.0 *_total_inst);
      Serial.println(lvl);
    
  
      uint32_t delta = 500;
      uint32_t delta_for;
      unsigned long time1, time2;
      time1 = millis();
      for (int i = 0; i < _total_inst; ++i)
      {
          if (instruments[i].finish)
            continue;
          delta_for =  instruments[i].time_current - _time_total;
          Serial.print("Delta_for: ");
          Serial.print(instruments[i].time_current);
          Serial.print(" - ");
          Serial.print(_time_total);
          Serial.print(" = ");
          Serial.println(delta_for);
          if(delta > delta_for)
              delta = delta_for;
      }
  
      for (int i = 0; i < _total_drums; ++i)
      {
          if (drums[i].finish)
            continue;
          delta_for = drums[i].time_current - _time_total;
          Serial.print("Delta_for: ");
          Serial.print(drums[i].time_current);
          Serial.print(" - ");
          Serial.print(_time_total);
          Serial.print(" = ");
          Serial.println(delta_for);
          if(delta > delta_for)
              delta = delta_for;
      }
      _frozen = true;
      _delay_time = (int)(delta * _fact);
      _time_total += delta;
      Serial.print(", Delta: ");
      Serial.println(delta);
    }

    _heating(delta_trash);
    Serial.print("Delay: ");
    Serial.println(_delay_time);
    //delay(_delay_time);
    Serial.print("Lvl : ");
    Serial.print(lvl);
    Serial.print(", Tiempo total: ");
    Serial.print(_time_total);

    if(!_frozen){
      float chunk = 0;
      for (int i = 0; i < _total_inst; ++i)
      {
          if (lvl >= 127){
              chunk = 127;
              lvl -= 127;
          }else{
              chunk = lvl;
              lvl = 0;   
          }
          instruments[i].play(_time_total, chunk);
      }
  
      for (int i = 0; i < _total_drums; ++i)
      {
          drums[i].play(_time_total, 0);
      }
    }


    //Serial.print("Esperando: ");
    //Serial.println(delta);
}

void Player::_heating(int delta_trash){
  _delay_time -= _delay_time - delta_trash > 13 ? delta_trash: _delay_time;
  if(_delay_time > 0 && _delay_time < 14){
    delay(14);
    _delay_time -= 14;
  }
  else{
    delay(_delay_time);
    _delay_time -= _delay_time;
    _frozen = false;
  }
  

}

// definition for float map function 
// from: https://forum.arduino.cc/index.php?topic=3922.0 (post #2)
float Player::_mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
