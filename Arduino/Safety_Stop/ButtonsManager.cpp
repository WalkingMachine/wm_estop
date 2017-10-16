#include "ButtonsManager.h"

float voltageBat_GV = 21.0F; //valeur initial pour ignorer les fauce lecture au demerage

void initButtons(){
  pinMode(START_BUTTON, INPUT);
  pinMode(SAFETY_BUTTON, INPUT);
}

bool start_button_handle() {
  static bool start_last_reading = false;
  static bool start_enter = true;
  static unsigned long start_last_debounce_time = 0;

  //read start button state
  bool reading = digitalRead(START_BUTTON);

  //check if bounce
  if((millis() - start_last_debounce_time) > DEBOUNCE_DELAY)
  {
    start_enter = true;
  }

  if (reading != start_last_reading && start_enter){
    start_last_debounce_time = millis();
    start_enter = false;

    //update start button state
    start_last_reading = reading;
  }
  return start_last_reading;
}

bool stop_button_handle(){
  static bool stop_last_reading;
  static bool stop_enter = true;
  static unsigned long stop_last_debounce_time = 0;

  //read stop button
  bool reading = digitalRead(SAFETY_BUTTON);

  //check if bounce
  if((millis() - stop_last_debounce_time) > DEBOUNCE_DELAY) {
    stop_enter = true;
  }

  if (reading != stop_last_reading && stop_enter) {
    stop_last_debounce_time = millis();
    stop_enter = false;

    //update start button state
    stop_last_reading = reading;
  }
  return stop_last_reading;
}

bool batLow_handle()
{
  float voltageBat = (analogRead(ANALOG_BAT_PIN) / 1023.0F) * 5.0F * ANALOG_GAIN; //0-1023 -> 0-21V
  voltageBat_GV = voltageBat_GV * 0.95F + voltageBat * 0.05F;//moyene, poid de la nouvelle lecture 10%

  if(voltageBat_GV < BAT_LOW_V)
  {
    return true;  
  }
  else
  {
    return false;  
  }
}

bool batCritical_handle()
{
  float voltageBat = (analogRead(ANALOG_BAT_PIN) / 1023.0F) * 5.0F * ANALOG_GAIN; //0-1023 -> 0-21V
  voltageBat_GV = voltageBat_GV * 0.95F + voltageBat * 0.05F;//moyene, poid de la nouvelle lecture 10%
  
  if(voltageBat_GV < BAT_CRITICAL_V)
  {
    return true;  
  }
  else
  {
    return false;  
  }
}
