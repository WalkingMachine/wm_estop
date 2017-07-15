#include "ButtonsManager.h"

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
