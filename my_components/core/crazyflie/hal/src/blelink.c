/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * wifilink.c: ESP32 Wi-Fi implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "blelink.h"
#include "ble_esp32.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "BLELINK"
#include "debug_cf.h"
#include "static_mem.h"

#define BLE_ACTIVITY_TIMEOUT_MS (1000)

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static uint8_t sendBuffer[64];

static BLEPacket bleIn;
static CRTPPacket p;
static uint32_t lastPacketTick;

static int blelinkSendPacket(CRTPPacket *p);
static int blelinkSetEnable(bool enable);
static int blelinkReceiveCRTPPacket(CRTPPacket *p);

STATIC_MEM_TASK_ALLOC(blelinkTask, USBLINK_TASK_STACKSIZE);

static float rch, pch, ych;
static uint16_t tch;

static bool blelinkIsConnected(void)
{
    return bleConnectedClients();
}

static struct crtpLinkOperations blelinkOp = {
    .setEnable         = blelinkSetEnable,
    .sendPacket        = blelinkSendPacket,
    .receivePacket     = blelinkReceiveCRTPPacket,
    .isConnected       = blelinkIsConnected,
};

static bool detectOldVersionApp(BLEPacket *in)
{

    if ((in->data)[0] != 0x00 && in->size == 11) { //12-1
        if ((in->data)[0] == 0x80 && (in->data)[9] == 0x00 && (in->data)[10] == 0x00 && (in->data)[11] == 0x00) {
            return true;
        }
    }

    return false;
}

static void blelinkTask(void *param)
{
    while (1) {
        /* command step - receive  03 Fetch a wifi packet off the queue */
        bleGetDataBlocking(&bleIn);
        lastPacketTick = xTaskGetTickCount();
#ifdef CONFIG_ENABLE_LEGACY_APP

        if (detectOldVersionApp(&bleIn)) {
            rch  = (1.0) * (float)(((((uint16_t)bleIn.data[1] << 8) + (uint16_t)bleIn.data[2]) - 296) * 15.0 / 150.0); //-15~+15
            pch  = (-1.0) * (float)(((((uint16_t)bleIn.data[3] << 8) + (uint16_t)bleIn.data[4]) - 296) * 15.0 / 150.0); //-15~+15
            tch  = (((uint16_t)bleIn.data[5] << 8) + (uint16_t)bleIn.data[6]) * 59000.0 / 600.0;
            ych  = (float)(((((uint16_t)bleIn.data[7] << 8) + (uint16_t)bleIn.data[8]) - 296) * 15.0 / 150.0); //-15~+15
            p.size = bleIn.size + 1 ; //add cksum size
            p.header = CRTP_HEADER(CRTP_PORT_SETPOINT, 0x00); //head redefine

            memcpy(&p.data[0], &rch, 4);
            memcpy(&p.data[4], &pch, 4);
            memcpy(&p.data[8], &ych, 4);
            memcpy(&p.data[12], &tch, 2);
        } else
#endif
        {
            /* command step - receive  04 copy CRTP part from packet, the size not contain head */
            p.size = bleIn.size - 1;
            memcpy(&p.raw, bleIn.data, bleIn.size);
        }

        /* command step - receive 05 send to crtpPacketDelivery queue */
        xQueueSend(crtpPacketDelivery, &p, 0) ;
    }

}

static int blelinkReceiveCRTPPacket(CRTPPacket *p)
{
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
        ledseqRun(&seq_linkUp);
        return 0;
    }

    return -1;
}

static int blelinkSendPacket(CRTPPacket *p)
{
    int dataSize;

    ASSERT(p->size < SYSLINK_MTU);

    sendBuffer[0] = p->header;

    if (p->size <= CRTP_MAX_DATA_SIZE) {
        memcpy(&sendBuffer[1], p->data, p->size);
    }

    dataSize = p->size + 1;


    /*ledseqRun(&seq_linkDown);*/

    return bleSendData(dataSize, sendBuffer);
}

static int blelinkSetEnable(bool enable)
{
    return 0;
}

/*
 * Public functions
 */

void blelinkInit()
{
    if (isInit) {
        return;
    }


    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    STATIC_MEM_TASK_CREATE(blelinkTask, blelinkTask,BLELINK_TASK_NAME,NULL, BLELINK_TASK_PRI);

    isInit = true;
}

bool blelinkTest()
{
    return isInit;
}

struct crtpLinkOperations *blelinkGetLink()
{
    return &blelinkOp;
}
