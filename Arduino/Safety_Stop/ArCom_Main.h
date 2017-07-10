//
// Created by lucas on 09/07/17.
//

#ifndef WM_ARDUINOLIBRARY_ARCOM_MAIN_H
#define WM_ARDUINOLIBRARY_ARCOM_MAIN_H

#define MESSAGE_SIZE        8   //The total number of bytes in a message
#define ID_POSITION         0   //Index of the first ID byte
#define DATA_POSITION       2   //Index of the first DATA byte
#define CHECKSUM_POSITION   7   //Index of the checksum byte

typedef unsigned char  T_Message[MESSAGE_SIZE];

/**
 * @param theMessage an RS232 message of ArCom form
 * @return the checksum
 */
unsigned char calculateChecksum(T_Message theMessage);

/**
 * @param theMessage an RS232 message of ArCom form
 * @return if the checksum is correct.
 */
bool verifyChecksum(T_Message theMessage);

/**
 * @param theMessage an RS232 message of ArCom form
 * @return the id of the message in parameter
 */
int getID(T_Message theMessage);

/**
 * @param theMessage an RS232 message of ArCom form
 * @return the data integer of the message in parameter
 */
long getInteger(T_Message theMessage);

/**
 * @param ID of the data to send
 * @param data to send
 * @return generated message for serial sent
 */
void generateMessageInteger(int ID, long data, T_Message theMessage);

#endif