#ifndef __Protothreads_H
#define __Protothreads_H 
#pragma once
#include "stdio.h"
#include "main.h"
#include "string.h"

#define MAXTASKS 9

extern volatile unsigned int Timer[MAXTASKS];

typedef int32_t usbDataWriteType;

#define USB_BEGIN_FLAG ((usbDataWriteType)0xEEEEEEEE)
#define USB_END_FLAG   ((usbDataWriteType)0xDDDDDDDD)


#pragma pack(push, 1)
typedef struct
{
    usbDataWriteType usbBegin;

    usbDataWriteType t_Index;

    usbDataWriteType signalA0;
    usbDataWriteType signalA1;
    usbDataWriteType signalA2;
    usbDataWriteType signalA3;
    usbDataWriteType signalA4;
    usbDataWriteType signalA5;

    usbDataWriteType signalB0;
    usbDataWriteType signalB1;
    usbDataWriteType signalB2;
    usbDataWriteType signalB3;
    usbDataWriteType signalB4;
    usbDataWriteType signalB5;

    usbDataWriteType usbEnd;
} Pc2CdcMessage_t;
#pragma pack(pop)


int USB_RX_Data(uint8_t *buf, uint16_t len);

int Printf_Task(void);
int LED_Task(void);
int OLED_Task(void);
int USB_Task(void);
void USB_TX_Task(void);
void USB_RX_Task(void);
int Current_Task(void);
#endif



