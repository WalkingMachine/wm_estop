#ifndef WM_BUTTONMANAGER_H
#define WM_BUTTONMANAGER_H

#include <Arduino.h>

#define SAFETY_BUTTON   2       //PIN for safety button
#define START_BUTTON    3       //PIN for start button
#define ANALOG_BAT_PIN  0       //PIN for analog read batery

#define ANALOG_GAIN     5.62    //Gain of the Analog Battery Read
#define BAT_LOW_V       17      //Low voltage of the battery (~5% capacity)
#define BAT_CRITICAL_V  15      //Extinction voltage (0% capacity)

#define DEBOUNCE_DELAY 500      //delay of bounce for debouncers

void initButtons();
bool start_button_handle();
bool stop_button_handle();
bool batLow_handle();
bool batCritical_handle();

#endif
