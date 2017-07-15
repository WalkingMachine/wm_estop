#ifndef WM_BUTTONMANAGER_H
#define WM_BUTTONMANAGER_H

#include <Arduino.h>

#define SAFETY_BUTTON   2       //PIN for safety button
#define START_BUTTON    3       //PIN for start button

#define DEBOUNCE_DELAY 500      //delay of bounce for debouncers

void initButtons();
bool start_button_handle();
bool stop_button_handle();

#endif
