/*
 * ble_esp32.cpp
 *
 *  Created on: 26 Sep 2022
 *      Author: Aviv
 */

#define DEBUG_MODULE  "BLE"

extern "C"{
#include "ble_esp32.h"
}
#include "config.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEService.h>
#include <BLEAdvertising.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queuemonitor.h"
#include "stm32_legacy.h"
#include "debug_cf.h"

#define BLE_SERVER_BUFSIZE      128

static const char* DEVICE_NAME = "Crazyflie";

static const char* SERVICE_UUID =	"00000201-1C7F-4F9E-947B-43B7C00A9A08";
static const char* CTRP_UUID =		"00000202-1C7F-4F9E-947B-43B7C00A9A08";
static const char* CTRPUP_UUID =	"00000203-1C7F-4F9E-947B-43B7C00A9A08";
static const char* CRTPDOWN_UUID =	"00000204-1C7F-4F9E-947B-43B7C00A9A08";

static bool oldDeviceConnected = false;
static bool deviceConnected = false;

extern "C"{
void bleInit(void);
bool bleTest(void);
bool bleGetDataBlocking(BLEPacket *in);
bool bleSendData(uint32_t size, uint8_t* data);
}

static BLEServer *pServer = NULL;
static BLECharacteristicCallbacks *pCallbacks;
static BLECharacteristic *ctrpChar,*ctrpUpChar,*ctrpDownChar;

static char tx_buffer[BLE_SERVER_BUFSIZE];

static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;
static BLEPacket inPacket;
static BLEPacket outPacket;

static void bleHandel(void*);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
		deviceConnected = true;
		DEBUG_PRINT_LOCAL("Connected!");
    };

    void onDisconnect(BLEServer* pServer) {
//		pServer->startAdvertising();
    	deviceConnected = false;
		DEBUG_PRINT_LOCAL("Disconnected!");
    }
};

class MyCharCallbacks: public BLECharacteristicCallbacks{
	void onWrite(BLECharacteristic* pCharacteristic)override{
//		ESP_LOGI("BLE","Received %d of data\n",pCharacteristic->getLength());
		uint8_t len;
		if(pCharacteristic == ctrpChar){
			len = pCharacteristic->getValue().size();
			memcpy(&inPacket.data,pCharacteristic->getData(),len);
			inPacket.data[len] = 0;// Null-terminate whatever we received and treat like a string...
			inPacket.size = len;
			xQueueSend(udpDataRx, &inPacket, M2T(200));
//			ESP_LOGI("BLE","Received %d of data\n",pCharacteristic->getLength());
		}else if(pCharacteristic == ctrpUpChar){
			//TODO: implement
		}
	}
};

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = (unsigned char*)data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }

    return cksum;
}

static void udp_server_tx_task(void *pvParameters)
{
    while (TRUE) {
        if(!pServer) {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, 5) == pdTRUE) && deviceConnected) {
            memcpy(tx_buffer, outPacket.data, outPacket.size);
            tx_buffer[outPacket.size] =  calculate_cksum(tx_buffer, outPacket.size);
            tx_buffer[outPacket.size + 1] = 0;
            ctrpDownChar->setValue((uint8_t*)tx_buffer, outPacket.size);
            ctrpDownChar->notify();
#ifdef DEBUG_UDP
            DEBUG_PRINT_LOCAL("Send data to");
            for (size_t i = 0; i < outPacket.size + 1; i++) {
                DEBUG_PRINT_LOCAL(" data_send[%d] = %02X ", i, tx_buffer[i]);
            }
#endif
        }
    }
}

void bleInit(void){
	if(pServer)
		return;
	BLEDevice::init(DEVICE_NAME);
	pServer = BLEDevice::createServer();
	pCallbacks = new MyCharCallbacks();
	pServer->setCallbacks(new MyServerCallbacks());
	BLEService *pService = pServer->createService(SERVICE_UUID);

	ctrpChar = pService->createCharacteristic(
		CTRP_UUID,
		BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
	);
	ctrpUpChar = pService->createCharacteristic(
		CTRPUP_UUID,
		BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
	);
	ctrpDownChar = pService->createCharacteristic(
		CRTPDOWN_UUID,
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
	);

    udpDataRx = xQueueCreate(5, sizeof(BLEPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(5, sizeof(BLEPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

	ctrpChar->addDescriptor(new BLE2902());
	ctrpDownChar->addDescriptor(new BLE2902());
	ctrpChar->setCallbacks(pCallbacks);
	ctrpUpChar->setCallbacks(pCallbacks);

	pService->start();

	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter

    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(bleHandel,"BLEHandel",UDP_TX_TASK_STACKSIZE,NULL,UDP_TX_TASK_PRI,NULL);
    BLEDevice::startAdvertising();
}

bool bleTest(void){
	return pServer;
}

bool bleGetDataBlocking(BLEPacket *in){
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(1);
    }; // Don't return until we get some data on the UDP

    return true;
};

bool bleSendData(uint32_t size, uint8_t *data){
    static BLEPacket outStage;
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

int bleConnectedClients(){
	return deviceConnected;
}

static void bleHandel(void* arg){
	while(true){
		if (!deviceConnected && oldDeviceConnected) {
			vTaskDelay(M2T(500)); // give the bluetooth stack the chance to get things ready
			pServer->startAdvertising(); // restart advertising
			oldDeviceConnected = deviceConnected;
		}
		// connecting
		if (deviceConnected && !oldDeviceConnected) {
			// do stuff here on connecting
			oldDeviceConnected = deviceConnected;
		}
		vTaskDelay(M2T(100));
	}
}
