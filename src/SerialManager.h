//
// Created by lucas on 11/07/17.
//

#ifndef PROJECT_SERIALMANAGER_H
#define PROJECT_SERIALMANAGER_H

#include "serial/SerialStream.h"
#include <ctime>
#include <string>
#include <iostream>

#define WATCHDOG 1000    //ms
#define SEND_DELAY 100    //ms

namespace wm {
	class SerialManager {
	public:
		//initialise serial connection
		SerialManager(std::string portName);

		//read last received message if exist
		bool serialHandler();

		//test watchdog
		bool watchDogHandler();

		//send current running status to board
		void sendStatus(bool status);

		//read if serial buffer contain data or not
		bool SerialAvailable();
	private:
		LibSerial::SerialStream serial_port;
		unsigned long lastMessageTime;
		unsigned long lastSendTime;
	};
}//namespace wm

#endif //PROJECT_SERIALMANAGER_H
