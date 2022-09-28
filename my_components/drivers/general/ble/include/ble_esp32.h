/*
 * ble_esp32.h
 *
 *  Created on: 26 Sep 2022
 *      Author: Aviv
 */

#ifndef MY_COMPONENTS_DRIVERS_GENERAL_BLE_INCLUDE_BLE_ESP32_H_
#define MY_COMPONENTS_DRIVERS_GENERAL_BLE_INCLUDE_BLE_ESP32_H_

#include <stdint.h>

#define BLE_RX_TX_PACKET_SIZE   (64)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[BLE_RX_TX_PACKET_SIZE];
} BLEPacket;

/**
 * Initialize the ble.
 *
 * @note Initialize CRTP link only if USE_CRTP_WIFI is defined
 */
void bleInit(void);

/**
 * Test the BLE status.
 *
 * @return true if the WIFI is initialized
 */
bool bleTest(void);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
//struct crtpLinkOperations * bleGetLink();

/**
 * Get data from rx queue with timeout.
 * @param[out] c  Byte of data
 *
 * @return true if byte received, false if timout reached.
 */
bool bleGetDataBlocking(BLEPacket *in);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If WIFI Crtp link is activated this function does nothing
 */
bool bleSendData(uint32_t size, uint8_t* data);

/**
 * Return number of connected clients
 * @return number of connected clients
 */
int bleConnectedClients();

#endif /* MY_COMPONENTS_DRIVERS_GENERAL_BLE_INCLUDE_BLE_ESP32_H_ */
