/*
 * UnitiBLE.h
 *
 *  Created on: 29 Jan 2020
 *      Author: Aviv
 */

#ifndef MAIN_INCLUDES_UNITIBLE_H_
#define MAIN_INCLUDES_UNITIBLE_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

typedef void (*callback_t)(const std::string&);
typedef void (*lock_callback_t)(bool);
typedef void (*command_callback_t)(uint8_t);
/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_MAIN,
    IDX_CHAR_VAL_MAIN,
    IDX_CHAR_CFG_MAIN,

    IDX_CHAR_SSID,
    IDX_CHAR_VAL_SSID,

	IDX_CHAR_PASSWORD,
	IDX_CHAR_VAL_PASSWORD,

	IDX_CHAR_RESTART,
	IDX_CHAR_VAL_RESTART,

	IDX_CHAR_LOCK,
	IDX_CHAR_VAL_LOCK,

	IDX_CHAR_CERT,
	IDX_CHAR_VAL_CERT,

	IDX_CHAR_TOPIC,
	IDX_CHAR_VAL_TOPIC,

	IDX_CHAR_MAC,
	IDX_CHAR_VAL_MAC,

    HRS_IDX_NB,
};

void Uniti_BLE_Init();
void Uniti_BLE_Start();
void Uniti_BLE_Stop();

void Uniti_BLE_setOnSSID(callback_t callback);
void Uniti_BLE_setOnPASSWORD(callback_t callback);
void Uniti_BLE_setOnConnected(callback_t callback);
void Uniti_BLE_setOnDisconnected(callback_t callback);
void Uniti_BLE_setOnCommand(command_callback_t callback);
void Uniti_BLE_setOnLock(lock_callback_t callback);
void Uniti_BLE_setOnCertUpdate(callback_t callback);
void Uniti_BLE_setOnTopicUpdate(callback_t callback);
#endif /* MAIN_INCLUDES_UNITIBLE_H_ */
