/*
 * ATmega328P based safety stop
 * Service: _safety_stop_srv
 * Topic: start_button
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>


#define SAFETY_BUTTON   2       //PIN for safety button
#define START_BUTTON    3       //PIN for start button
#define LED_PIN         4       //PIN for LED feedback
#define BUZZER_PIN      5       //PIN for BUZZER feedback
#define POWER_CTRL      6       //PIN for power relay

#define SHORT_INTERVAL 100      //LED short blink interval
#define REGULAR_INTERVAL 500    //LED regular blink interval
#define LONG_INTERVAL 1000      //LED long blink interval

#define BUZZ_TIME 200           //BUZZER TIME
#define BUZZ_TONE 1047          //BUZZER TONE

#define DEBOUNCE_DELAY 500      //delay of bounce for debouncers

#define TIMEOUT_FEEDBACK 500   //timeout for service caller feedback

#define PUBLISH_INTERVAL 200


typedef enum{
	RUN,
	STOP,
	LAPTOP_FEEDBACK_PENDING,
	START_PENDING,
  START_BACK,
  WAIT_START_RAISED,
	SHUTDOWN
}T_State;



//System state
T_State system_state;

//create ros handle
ros::NodeHandle  nh;

//generate start message
std_msgs::Bool start_msg;

//generate start button publisher
ros::Publisher start_button("start_button_msg", &start_msg);

//initialise service client
ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> _safety_stop_srv("safety_stop_srv");

bool start_button_handle();
bool stop_button_handle();
bool stop_init;
void led_handler();

void setup(){
	//set pinout
	pinMode(START_BUTTON, INPUT);
	pinMode(SAFETY_BUTTON, INPUT);
	pinMode(POWER_CTRL, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
  
	//read initial state of STOP switch /!\ NNC
  bool bSafetyState = digitalRead(SAFETY_BUTTON);
	if(bSafetyState){
    digitalWrite(POWER_CTRL, LOW);
    system_state = RUN;
	}else{
    digitalWrite(POWER_CTRL, HIGH);
    system_state = START_PENDING;
	}

  tone(BUZZER_PIN, BUZZ_TONE, BUZZ_TIME);

	//ros initialisations
	nh.initNode();
	nh.advertise(start_button);
	nh.serviceClient(_safety_stop_srv);

	while(!nh.connected()){
    bSafetyState = digitalRead(SAFETY_BUTTON);
    if(bSafetyState){
      digitalWrite(POWER_CTRL, LOW);
      system_state = RUN;
    }else{
      digitalWrite(POWER_CTRL, HIGH);
      system_state = START_PENDING;
    }
	  nh.spinOnce();
	}
}

void loop() {
	static unsigned long firstServiceCallTime = 0 ;
  static unsigned long publisherTime = 0 ;
	static bool bFeedbackReceived = false;
	static bool bTimeout = false;

	//------ input handler ------

	bool bStopPressed = !stop_button_handle();
	bool bStartPressed = start_button_handle();

	//------ StateMachine ------

	switch(system_state){
		case RUN:                                   //System is running normally, waiting for press on STOP button.
			if(bStopPressed){
				system_state = STOP;
			}if(bStartPressed){
        system_state = START_BACK;
			}
			break;

		case STOP:                                  //System go in stop mode, call a ROS service for inform main system.
			system_state = LAPTOP_FEEDBACK_PENDING;
			break;

		case LAPTOP_FEEDBACK_PENDING:               //System wait for main system answer.
			if(bFeedbackReceived){
				system_state = START_PENDING;           //IF answer, will wait for press on START button.
			}else if(bTimeout){
				system_state = SHUTDOWN;                //IF timeout, go to shutdown.
			}
			break;

		case START_PENDING:                         //System is waiting for press on START button for go back in normal running mode.
			if(bStartPressed && !bStopPressed){
				system_state = START_BACK;
			}
			break;

   case START_BACK:
      if(bStopPressed){
        system_state = STOP; 
      }else{
        system_state = WAIT_START_RAISED;
      }
      break;

    case WAIT_START_RAISED:
      if(bStopPressed){
        system_state = STOP; 
      }else if(!bStartPressed){
        system_state = RUN;
      }
      break;

		case SHUTDOWN:                              //System is going to shutdown.
		  if(bStartPressed && !bStopPressed){
				system_state = RUN;
			}
			break;
	}

	//------ output handlers ------

	//LED Handle
	led_handler();

	//Power Relay Control
  digitalWrite(POWER_CTRL, system_state == SHUTDOWN);
	
  //ros initialise service caller
  if(system_state == START_BACK){
    std_srvs::SetBool::Request req;
      std_srvs::SetBool::Response res;

      //write message
      req.data = false;

      //call RESTART service
      _safety_stop_srv.call(req, res);
  }

	//ros initialise service caller
	if(system_state == STOP){
		//initialise timeout
		firstServiceCallTime = millis();
		bFeedbackReceived = false;
		bTimeout = false;
	}

	//ros service caller
	if(system_state == STOP || system_state == LAPTOP_FEEDBACK_PENDING){
		if((millis() - firstServiceCallTime) > TIMEOUT_FEEDBACK){
			bTimeout = true;
		}else{
			//create message
			std_srvs::SetBool::Request req;
			std_srvs::SetBool::Response res;

			//write message
			req.data = true;

			//call STOP service
			_safety_stop_srv.call(req, res);

      delay(10);

			//if success, raise the flag
			if(res.success) {
				bFeedbackReceived = true;
		  }
		}
	}

 //ros publisher
  if((millis() - publisherTime) > PUBLISH_INTERVAL && system_state != SHUTDOWN ){
    publisherTime = millis();
    start_msg.data = system_state == RUN || system_state == START_BACK || system_state == WAIT_START_RAISED;
    start_button.publish(&start_msg);
  }
  
	nh.spinOnce();
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
  static bool stop_last_reading = stop_init;
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

		case STOP: case LAPTOP_FEEDBACK_PENDING:
      timeHigh = timeLow = REGULAR_INTERVAL;
			break;

		case START_PENDING:
		  timeHigh = timeLow = SHORT_INTERVAL;
      break;
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

