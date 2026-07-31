/* usbd_cdc_if.c - CDC ACM glue: RX bytes go to the command console's ring
 * buffer (interrupt context); the console parses and replies from the main
 * loop. Replaces the ST example's UART-bridge implementation. */
#include "usbd_cdc_if.h"
#include "console.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Hosts require GET_LINE_CODING to succeed; content is irrelevant for us. */
static uint8_t linecoding[7] = {0x00, 0xC2, 0x01, 0x00,  /* 115200 baud */
                                0x00, 0x00, 0x08};       /* 1 stop, no par, 8b */

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
  return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd) {
  case CDC_SET_LINE_CODING:
    if (length >= 7) memcpy(linecoding, pbuf, 7);
    break;
  case CDC_GET_LINE_CODING:
    if (length >= 7) memcpy(pbuf, linecoding, 7);
    break;
  default:                       /* remaining class requests need no action */
    break;
  }
  return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  console_rx(Buf, *Len);         /* copy into ring buffer (IRQ context) */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  (void)Buf; (void)Len; (void)epnum;
  return USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Send Len bytes; returns USBD_BUSY if the previous transfer is running. */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) return USBD_FAIL;
  if (hcdc->TxState != 0) return USBD_BUSY;
  if (Len > APP_TX_DATA_SIZE) Len = APP_TX_DATA_SIZE;
  memcpy(UserTxBufferFS, Buf, Len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
