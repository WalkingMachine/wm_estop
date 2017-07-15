/*
 * ATmega328P based safety stop
 * Service: _safety_stop_srv
 * Topic: start_button
 */
 
#include "ButtonsManager.h"

#define LED_PIN         4       //PIN for LED feedback
#define BUZZER_PIN      5       //PIN for BUZZER feedback
#define POWER_CTRL      6       //PIN for power relay

#define SHORT_INTERVAL 100      //LED short blink interval
#define REGULAR_INTERVAL 500    //LED regular blink interval
#define LONG_INTERVAL 1000      //LED long blink interval

#define BUZZ_TIME 200           //BUZZER TIME
#define BUZZ_TONE 1047          //BUZZER TONE

#define TIMEOUT_FEEDBACK 1500   //timeout for service caller feedback

#define PUBLISH_INTERVAL 200

typedef enum{
	RUN,
	STOP,
	SHUTDOWN
}T_State;

//System state
T_State system_state;
bool bDataToRead;
bool bStateReceived;
bool connection_lost;

bool start_button_handle();
bool stop_button_handle();
void led_handler();

void setup(){
  Serial.begin(9600);
  
  //init secure button library
  initButtons();

  //set pinout
	pinMode(POWER_CTRL, OUTPUT);
	pinMode(LED_PIN, OUTPUT);

  //init state machine
  system_state = RUN;

  //init global var
  bDataToRead = false;
  bStateReceived = false;

  //Buzz
  tone(BUZZER_PIN, BUZZ_TONE, BUZZ_TIME);
}

void loop() {
  static unsigned long lastSerialReception = 0;
  static unsigned long publisherTime = 0 ;


	//------ input handler ------

  //read buttons
 	bool bStopPressed = !stop_button_handle();
	bool bStartPressed = start_button_handle();

  //check reading feedback
  if(bDataToRead){
    bDataToRead = false;
    if((system_state == RUN) == bStateReceived){
      lastSerialReception = millis();
    }
  }
  
  //manage timeout
  if((millis() - lastSerialReception) > TIMEOUT_FEEDBACK){
    connection_lost = true;
  }else if(lastSerialReception == 0){
    connection_lost = true;
  }else{
    connection_lost = false;
  }

	//------ StateMachine ------

	switch(system_state){
		case RUN:                                   //System is running normally, waiting for press on STOP button.
			if(bStopPressed){
				system_state = STOP;
			}
			break;

		case STOP:                         //System is waiting for press on START button for go back in normal running mode.
			if(bStartPressed && !bStopPressed){
				system_state = RUN;
			}
			break;

		case SHUTDOWN:                              //System is going to shutdown.
		  if(bStartPressed && !bStopPressed){
				system_state = RUN;
			}
			break;
	}

  //------ force states ------

  if(system_state != RUN && connection_lost){
    system_state = SHUTDOWN;
  }

	//------ output handlers ------

	//LED Handle
	led_handler();

	//Power Relay Control
  digitalWrite(POWER_CTRL, system_state == SHUTDOWN);
 
  //ros publisher
  if((millis() - publisherTime) > PUBLISH_INTERVAL){
    publisherTime = millis();
    Serial.write(system_state == RUN);
  }
}

void serialEvent() {
  if(Serial.available()){
    byte received = Serial.read();
    bDataToRead = true;
    bStateReceived = received == 1;
  }
}

void led_handler(){
	static unsigned long led_last_time = 0;
  static bool ledOn = false;
  int timeNow = millis() - led_last_time;
	int timeHigh = 0;
  int timeLow = 0;
  
	switch(system_state){
		case RUN:
      timeLow = LONG_INTERVAL;
      timeHigh = SHORT_INTERVAL;
			break;

		case STOP:
		  timeHigh = timeLow = SHORT_INTERVAL;
      break;

    case SHUTDOWN:
      timeHigh = LONG_INTERVAL;
      timeLow = SHORT_INTERVAL;
      break;
	}

  if(connection_lost){
    timeHigh = timeLow = REGULAR_INTERVAL;
  }
  
  if(timeNow >= timeHigh && ledOn){
    led_last_time = millis();
    digitalWrite(LED_PIN, HIGH);
    ledOn = false;
  }else if(timeNow >= timeLow && !ledOn){
    led_last_time = millis();
    digitalWrite(LED_PIN, LOW);
    ledOn = true;
  }else if(timeHigh == 0){
    digitalWrite(LED_PIN, HIGH);
  }
}

