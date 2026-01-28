#include "USB.h"
#include "control_simple.h"
#include "adc.h"
#include "stdio.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"    // 为了 CDC_Transmit_HS

extern USBD_HandleTypeDef hUsbDeviceHS;

volatile unsigned int Timer[MAXTASKS];

// ====== 这些外部变量/结构你工程里必须有声明（没有就要加对应头文件�?extern�?=====
extern volatile int32_t Encoder[3];  // 你代码里�?Encoder[0..2]
extern struct { int32_t PWM; } MotA;    // 按你实际 MotA 定义改（这里只是示意�?
// =======================================================================

// 按上位机格式的收发包
static Pc2CdcMessage_t usb_tx = {0};
static Pc2CdcMessage_t usb_rx = {0};

static usbDataWriteType tIndex = 0;

// 这里保存上位机下发的三个值（你原来叫 Encoder0/1/2�?
int32_t Encoder0, Encoder1, Encoder2;

void USB_RX_Task(void)   // 下位�?-> 上位机：发�?
{
	
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return;

    usb_tx.usbBegin = USB_BEGIN_FLAG;
    usb_tx.t_Index  = tIndex++;

    // ===== A �?6 个信号（按你原来 wooden_tx.EnA~EnF 的含义去映射�?====
    usb_tx.signalA0 = (usbDataWriteType)((int32_t)ControlSimple_GetCurrentPwmCmd());
    usb_tx.signalA1 = (usbDataWriteType)ADCS.Cur[1];
    usb_tx.signalA2 = (usbDataWriteType)((int32_t)ControlSimple_GetCurrentRefmAClamped());
    usb_tx.signalA3 = -(usbDataWriteType)ADCS.Cur[0];
    usb_tx.signalA4 = 0;
    usb_tx.signalA5 = 0;

    // ===== B �?6 个信号（你原�?CurA~CurF 现在全写 Encoder[0]，这里先给示例）=====
    usb_tx.signalB0 = (usbDataWriteType)((int32_t)ControlSimple_GetCurrentPIIntegrator());
//    usb_tx.signalB1 = -(usbDataWriteType)ADCS.Cur[0];;
//    usb_tx.signalB2 = (usbDataWriteType)((int32_t)ControlSimple_GetCurrentRefmAClamped());
    usb_tx.signalB3 = 0;
    usb_tx.signalB4 = 0;
    usb_tx.signalB5 = 0;

    usb_tx.usbEnd = USB_END_FLAG;

    CDC_Transmit_HS((uint8_t*)&usb_tx, sizeof(Pc2CdcMessage_t));
}

//void USB_TX_Task(void)   // 上位�?-> 下位机：接收
//{
//    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return;

//    if (usb_rx_data((uint8_t*)&wooden_rx,sizeof(wooden_t))==1)

//    {
//						TargetBusy=1;
//        // 包头包尾校验，避免错�?
//        if (usb_rx.usbBegin != USB_BEGIN_FLAG) return;
//        if (usb_rx.usbEnd   != USB_END_FLAG)   return;

//        // 你原来是：Encoder0 = wooden_rx.EnB; Encoder1 = EnC; Encoder2 = EnD;
//        // 现在按同位置映射：这里我�?A1/A2/A3（你也可以按上位机实际含义调整）
//        Encoder0 = (int32_t)usb_rx.signalA1;
//        Encoder1 = (int32_t)usb_rx.signalA2;
//        Encoder2 = (int32_t)usb_rx.signalA3;
//					TargetBusy=0;
//    }
//}











