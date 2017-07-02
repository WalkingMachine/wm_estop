/*
 * ATmega328P based safety stop
 * Service: safety_stop_srv
 * Topic: start_button
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

typedef enum{
  RUNNING =1,
  SAFETY_STOP,
  REQUEST_PENDING,
  START_PENDING,
}State;

const float V_MAX = 26;
const float V_MIN = 22;

const int START_BUTTON = 2;
const int SAFETY_BUTTON = 4;
const int POWER_CTRL = 3;
const int LED_PIN = 13;

const int REGULAR_INTERVAL = 500;
const int LONG_INTERVAL = 1000;
const int NUMBER_OF_TRY = 3;
const int DEBOUNCE_DELAY = 500;

bool start_last_reading = false;
bool stop_last_reading;
bool stop_enter = true;
bool start_enter = true;
unsigned long stop_last_debounce_time = 0;
unsigned long start_last_debounce_time = 0;
unsigned long led_last_time = 0;

int power_state = HIGH;
State system_state;

ros::NodeHandle  nh;

ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> safety_stop_srv("safety_stop_srv");

std_msgs::Bool start_msg;
ros::Publisher start_button("start_button_msg", &start_msg);

void start_button_handle();
void safety_stop_handler();
void led_handler(const int time);

void setup()
{
  pinMode(START_BUTTON, INPUT);
  pinMode(SAFETY_BUTTON, INPUT);
  pinMode(POWER_CTRL, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(POWER_CTRL, power_state);

  stop_last_reading = digitalRead(SAFETY_BUTTON);
  if(stop_last_reading == HIGH)
  {
    system_state = RUNNING;
  }
  else
  {
    system_state = SAFETY_STOP;
  }
  
  nh.initNode();
  nh.advertise(start_button);
  nh.serviceClient(safety_stop_srv);
  
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Safety stop startup complete");
}

void loop()
{
  
  if(system_state == RUNNING)
  { 
    start_button_handle();
    safety_stop_handler();
    led_handler(REGULAR_INTERVAL);
  }
  
  if(system_state == SAFETY_STOP)
  {
    digitalWrite(LED_PIN, HIGH);
    safety_stop_handler();
  }
  
  if(system_state == START_PENDING)
  {
    start_button_handle();
    safety_stop_handler();
    led_handler(LONG_INTERVAL);
  }
  
  digitalWrite(POWER_CTRL, power_state);
  nh.spinOnce();
}

void start_button_handle()
{
  bool reading = digitalRead(START_BUTTON);
  
  if((millis() - start_last_debounce_time) > DEBOUNCE_DELAY)
  {
    start_enter = true;
  }
  
  if (start_last_reading != reading && start_enter){
    
    start_last_debounce_time = millis();
    start_enter = false;
    
    start_last_reading = reading;
    if(reading){
      start_msg.data = reading;
      start_button.publish(&start_msg);
      system_state = RUNNING;
      power_state = HIGH;
    }
  }
}

void safety_stop_handler()
{
  bool reading = digitalRead(SAFETY_BUTTON);
  
  if((millis() - stop_last_debounce_time) > DEBOUNCE_DELAY)
  {
    stop_enter = true;
  }
  
  if (reading != stop_last_reading && stop_enter)
  {
    stop_last_debounce_time = millis();
    stop_enter = false;
    
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    stop_last_reading = reading;
    req.data = reading;
    
    if(reading == HIGH)
    {
      system_state = START_PENDING;
      safety_stop_srv.call(req, res);
    }
    else
    {
      system_state = REQUEST_PENDING;
      for(int i=0; i<NUMBER_OF_TRY; i++)
      {
        safety_stop_srv.call(req, res);
        if(res.success)
        {
          system_state = SAFETY_STOP;
          power_state = HIGH;
          break;
        }
        delay(10);
      }
    }
    
    if(system_state == REQUEST_PENDING)
    {
      system_state = SAFETY_STOP;
      power_state = LOW;
    }
  }
}

void led_handler(const int time)
{
  
  if (millis() - led_last_time >= time)
  {
    led_last_time = millis();
    digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN));
  }
}

void voltage_check()
{
  
}

