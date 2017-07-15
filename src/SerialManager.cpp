//
// Created by lucas on 11/07/17.
//

#include "SerialManager.h"
namespace wm {
	SerialManager::SerialManager(std::string portName) {
		lastMessageTime = 0;
		lastSendTime = 0;

		using namespace LibSerial;

		// Open the serial port.
		serial_port.Open(portName);
		if (!serial_port.good()) {
			std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] " << "Error: Could not open serial port."
			          << std::endl;
			exit(1);
		}


		// Set the baud rate of the serial port.
		serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
		if (!serial_port.good()) {
			std::cerr << "Error: Could not set the baud rate." << std::endl;
			exit(1);
		}

		// Set the number of data bits to 8.
		serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
		if (!serial_port.good()) {
			std::cerr << "Error: Could not set the data size to 8." << std::endl;
			exit(1);
		}

		// Disable parity.
		serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
		if (!serial_port.good()) {
			std::cerr << "Error: Could not disable the parity." << std::endl;
			exit(1);
		}

		// Set the number of stop bits.
		serial_port.SetNumOfStopBits(1);
		if (!serial_port.good()) {
			std::cerr << "Error: Could not set the number of stop bits." << std::endl;
			exit(1);
		}

		// Turn off hardware flow control.
		serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
		if (!serial_port.good()) {
			std::cerr << "Error: Could not use hardware flow control." << std::endl;
			exit(1);
		}
	}

	bool SerialManager::serialHandler() {
		static bool bLastState = false;
		if (serial_port.rdbuf()->in_avail() > 0) {
			char next_byte;
			serial_port.get(next_byte);
			bLastState = next_byte == 1;

			time_t timer;
			time(&timer);
			lastMessageTime = (unsigned long) timer * 1000;  //time in ms
		}
		return bLastState;
	}


	bool SerialManager::watchDogHandler() {
		static bool bLastState = false;
		time_t timer;
		time(&timer);
		//read actual system time
		unsigned long sysTime = (unsigned long) timer * 1000;  //time in ms

		if (sysTime - lastMessageTime > WATCHDOG) {
			return true;
		} else {
			return false;
		}
	}

	void SerialManager::sendStatus(bool status){
		static bool bLastState = false;
		time_t timer;
		time(&timer);
		//read actual system time
		unsigned long sysTime = (unsigned long) timer * 1000;  //time in ms

		if (sysTime - lastSendTime > SEND_DELAY) {
			char cChar = status;
			lastSendTime = sysTime;
			serial_port.write(&cChar, 1);
		}
	}

	bool SerialManager::SerialAvailable(){
		return serial_port.rdbuf()->in_avail() > 0;
	}

}//namespace wm