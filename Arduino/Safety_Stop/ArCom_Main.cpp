//
// Created by lucas on 09/07/17.
//

#include "ArCom_Main.h"

unsigned char calculateChecksum(T_Message theMessage){
	//initialise sum to zero
	unsigned char cCalculatedChecksum = 0;

	//calculate received data checksum by adding
	for(int iLoop = 0; iLoop < CHECKSUM_POSITION; iLoop++){
		cCalculatedChecksum += theMessage[iLoop];
	}

	//return checksum
	return cCalculatedChecksum;
}

bool verifyChecksum(T_Message theMessage){
	//calculate received checksum, and compare with calculated checksum
	return calculateChecksum(theMessage) == theMessage[CHECKSUM_POSITION];
}

int getID(T_Message theMessage){
	return theMessage[ID_POSITION] + (theMessage[ID_POSITION + 1] << 8);
}

long getInteger(T_Message theMessage){
	return theMessage[DATA_POSITION] + (theMessage[DATA_POSITION+1] << 8) + (theMessage[DATA_POSITION+2] << 16) + (theMessage[DATA_POSITION+3] << 24);
}

void generateMessageInteger(int ID, long data, T_Message theMessage){
	//write ID
	for(int index = 0; index < 2; index ++){
		theMessage[ID_POSITION + index] = (ID >> (index * 8)) & 0xFF;
	}

	//write data
	for(int index = 0; index < 4; index ++){
		theMessage[DATA_POSITION + index] = (data >> (index * 8)) & 0xFF;
	}

	//calculate and write checksum
	theMessage[CHECKSUM_POSITION] = calculateChecksum(theMessage);
}