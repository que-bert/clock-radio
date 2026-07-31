/* usbd_composite.c - CDC console + UAC1 speaker composite class.
 *
 * The core runs in its normal single-class mode: USBD_CoreFindEP() returns 0
 * for every endpoint, so all Setup/DataIn/DataOut land here and we dispatch by
 * interface / endpoint. CDC (interfaces 0-1, EPs 0x81/0x01/0x82) is forwarded
 * verbatim to the stock USBD_CDC class. Audio (interfaces 2-3, iso OUT 0x03,
 * mono 16-bit @ 32 kHz) is handled below and streamed to the DAC.
 */
#include "usbd_composite.h"
#include "usbd_cdc.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_core.h"
#include "audio.h"

/* ---- audio interface / endpoint layout ---- */
#define AUDIO_OUT_EP   0x03U
#define AUDIO_MPS      64U        /* 32 samples * 2 bytes: mono 16-bit @ 32 kHz */
#define AUDIO_AC_IF    2U         /* AudioControl interface number */
#define AUDIO_AS_IF    3U         /* AudioStreaming interface number */

#define COMPOSITE_CFG_DESC_SIZ  175U

/* UAC1 class-specific request codes / control selectors */
#define AUDIO_REQ_SET_CUR   0x01U
#define AUDIO_REQ_GET_CUR   0x81U
#define AUDIO_REQ_GET_MIN   0x82U
#define AUDIO_REQ_GET_MAX   0x83U
#define AUDIO_REQ_GET_RES   0x84U
#define AUDIO_CS_MUTE       0x01U
#define AUDIO_CS_VOLUME     0x02U

/* volume in UAC 1/256-dB units: 0 dB down to -50 dB in 1 dB steps */
#define AUDIO_VOL_MIN   (-50 * 256)
#define AUDIO_VOL_MAX   0
#define AUDIO_VOL_RES   256

static uint8_t  audio_alt;                                   /* AS alt setting */
__ALIGN_BEGIN static uint8_t audio_rxbuf[AUDIO_MPS] __ALIGN_END;

static int16_t  audio_vol = 0;            /* current volume, 1/256 dB */
static uint8_t  audio_mute;
static uint8_t  audio_ctl[4];             /* EP0 data stage buffer */
static uint8_t  audio_ctl_pending;        /* SET_CUR awaiting its data */
static uint8_t  audio_ctl_cs;             /* which control it targets */

/* 10^(-dB/20) in Q15 for 0..-50 dB */
static const uint16_t db_att_q15[51] = {
    32767, 29204, 26028, 23197, 20675, 18426, 16422, 14636, 13045, 11626,
    10362,  9235,  8231,  7336,  6538,  5827,  5193,  4628,  4125,  3677,
     3277,  2920,  2603,  2320,  2067,  1843,  1642,  1464,  1304,  1163,
     1036,   924,   823,   734,   654,   583,   519,   463,   412,   368,
      328,   292,   260,   232,   207,   184,   164,   146,   130,   116,
      104,
};

static void audio_apply_gain(void)
{
    uint32_t idx = (uint32_t)(-audio_vol + 128) / 256u;   /* nearest dB */
    if (idx > 50u) idx = 50u;
    audio_usb_set_host_gain(audio_mute ? 0u : db_att_q15[idx]);
}

/* ------------------------------------------------------------------ */
/* Combined configuration descriptor: CDC (with IAD) + UAC1 speaker.  */
/* ------------------------------------------------------------------ */
__ALIGN_BEGIN static uint8_t Composite_CfgDesc[COMPOSITE_CFG_DESC_SIZ] __ALIGN_END =
{
  /* ---- Configuration ---- */
  0x09, USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(COMPOSITE_CFG_DESC_SIZ), HIBYTE(COMPOSITE_CFG_DESC_SIZ),
  0x04,                       /* bNumInterfaces: CDC(2) + Audio(2) */
  0x01,                       /* bConfigurationValue */
  0x00,                       /* iConfiguration */
  0xC0,                       /* bmAttributes: self powered */
  USBD_MAX_POWER,             /* bMaxPower */

  /* ---- IAD grouping the two CDC interfaces ---- */
  0x08, 0x0B,                 /* bLength, bDescriptorType (IAD) */
  0x00,                       /* bFirstInterface */
  0x02,                       /* bInterfaceCount */
  0x02, 0x02, 0x01,           /* bFunction Class/SubClass/Protocol (CDC/ACM/AT) */
  0x00,                       /* iFunction */

  /* ---- CDC Communication interface (IF0) ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x00, 0x00, 0x01,           /* IF0, alt0, 1 endpoint */
  0x02, 0x02, 0x01,           /* CDC / ACM / AT commands */
  0x00,
  0x05, 0x24, 0x00, 0x10, 0x01,           /* CDC header functional desc */
  0x05, 0x24, 0x01, 0x00, 0x01,           /* call management functional desc */
  0x04, 0x24, 0x02, 0x02,                 /* ACM functional desc */
  0x05, 0x24, 0x06, 0x00, 0x01,           /* union functional desc */
  0x07, USB_DESC_TYPE_ENDPOINT,           /* CDC command (notify) endpoint */
  CDC_CMD_EP, 0x03,
  LOBYTE(CDC_CMD_PACKET_SIZE), HIBYTE(CDC_CMD_PACKET_SIZE),
  CDC_FS_BINTERVAL,

  /* ---- CDC Data interface (IF1) ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x01, 0x00, 0x02,           /* IF1, alt0, 2 endpoints */
  0x0A, 0x00, 0x00,           /* CDC data */
  0x00,
  0x07, USB_DESC_TYPE_ENDPOINT,           /* bulk OUT */
  CDC_OUT_EP, 0x02,
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00,
  0x07, USB_DESC_TYPE_ENDPOINT,           /* bulk IN */
  CDC_IN_EP, 0x02,
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00,

  /* ---- Audio Control interface (IF2) ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  AUDIO_AC_IF, 0x00, 0x00,    /* IF2, alt0, 0 endpoints */
  0x01, 0x01, 0x00,           /* AUDIO / AUDIOCONTROL */
  0x00,
  /* class-specific AC header: IT(12) + FU(9) + OT(9) + this(9) = 39 */
  0x09, 0x24, 0x01,           /* CS_INTERFACE, HEADER */
  0x00, 0x01,                 /* bcdADC 1.00 */
  LOBYTE(39), HIBYTE(39),     /* wTotalLength */
  0x01,                       /* bInCollection */
  AUDIO_AS_IF,                /* baInterfaceNr(1) */
  /* Input Terminal: USB streaming */
  0x0C, 0x24, 0x02,
  0x01,                       /* bTerminalID = 1 */
  0x01, 0x01,                 /* wTerminalType = USB streaming (0x0101) */
  0x00,                       /* bAssocTerminal */
  0x01,                       /* bNrChannels = 1 (mono) */
  0x00, 0x00,                 /* wChannelConfig */
  0x00,                       /* iChannelNames */
  0x00,                       /* iTerminal */
  /* Feature Unit: master mute + volume - advertising hardware volume makes
   * the host send full-scale samples and control loudness via SET_CUR,
   * which we apply before the 12-bit truncation (much better SNR) */
  0x09, 0x24, 0x06,
  0x03,                       /* bUnitID = 3 */
  0x01,                       /* bSourceID = Input Terminal (1) */
  0x01,                       /* bControlSize = 1 byte */
  0x03,                       /* bmaControls(master): mute | volume */
  0x00,                       /* bmaControls(ch1): none */
  0x00,                       /* iFeature */
  /* Output Terminal: speaker */
  0x09, 0x24, 0x03,
  0x02,                       /* bTerminalID = 2 */
  0x01, 0x03,                 /* wTerminalType = Speaker (0x0301) */
  0x00,                       /* bAssocTerminal */
  0x03,                       /* bSourceID = Feature Unit (3) */
  0x00,                       /* iTerminal */

  /* ---- Audio Streaming interface (IF3) alt0: zero bandwidth ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  AUDIO_AS_IF, 0x00, 0x00,
  0x01, 0x02, 0x00,           /* AUDIO / AUDIOSTREAMING */
  0x00,
  /* ---- alt1: one iso OUT endpoint ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  AUDIO_AS_IF, 0x01, 0x01,
  0x01, 0x02, 0x00,
  0x00,
  /* class-specific AS general */
  0x07, 0x24, 0x01,
  0x01,                       /* bTerminalLink = Input Terminal (1) */
  0x01,                       /* bDelay */
  0x01, 0x00,                 /* wFormatTag = PCM */
  /* Type I format */
  0x0B, 0x24, 0x02,
  0x01,                       /* FORMAT_TYPE_I */
  0x01,                       /* bNrChannels = 1 */
  0x02,                       /* bSubframeSize = 2 */
  0x10,                       /* bBitResolution = 16 */
  0x01,                       /* bSamFreqType = 1 discrete */
  (uint8_t)(AUDIO_USB_HZ & 0xFF),
  (uint8_t)((AUDIO_USB_HZ >> 8) & 0xFF),
  (uint8_t)((AUDIO_USB_HZ >> 16) & 0xFF),
  /* standard iso audio data endpoint (9 bytes) */
  0x09, USB_DESC_TYPE_ENDPOINT,
  AUDIO_OUT_EP,
  0x09,                       /* iso, adaptive, data */
  LOBYTE(AUDIO_MPS), HIBYTE(AUDIO_MPS),
  0x01,                       /* bInterval = 1 frame */
  0x00,                       /* bRefresh */
  0x00,                       /* bSynchAddress */
  /* class-specific iso data endpoint (7 bytes) */
  0x07, 0x25, 0x01,
  0x00,                       /* bmAttributes */
  0x00,                       /* bLockDelayUnits */
  0x00, 0x00,                 /* wLockDelay */
};

_Static_assert(sizeof(Composite_CfgDesc) == COMPOSITE_CFG_DESC_SIZ,
               "composite config descriptor length mismatch");

__ALIGN_BEGIN static uint8_t Composite_DeviceQualifier[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02, 0xEF, 0x02, 0x01, 0x40, 0x01, 0x00,
};

/* ------------------------------------------------------------------ */
/* Audio class requests (interfaces 2-3 / endpoint 0x03).             */
/* ------------------------------------------------------------------ */
/* class-specific (feature unit) mute/volume requests on the AC interface */
static uint8_t Audio_ClassReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t cs = HIBYTE(req->wValue);

  switch (req->bRequest) {
  case AUDIO_REQ_SET_CUR:
    if (req->wLength != 0U && req->wLength <= sizeof(audio_ctl)) {
      audio_ctl_pending = 1U;
      audio_ctl_cs = cs;
      (void)USBD_CtlPrepareRx(pdev, audio_ctl, req->wLength);
      return USBD_OK;
    }
    break;

  case AUDIO_REQ_GET_CUR:
    if (cs == AUDIO_CS_MUTE) {
      audio_ctl[0] = audio_mute;
      (void)USBD_CtlSendData(pdev, audio_ctl, 1U);
      return USBD_OK;
    }
    if (cs == AUDIO_CS_VOLUME) {
      audio_ctl[0] = (uint8_t)(audio_vol & 0xFF);
      audio_ctl[1] = (uint8_t)((audio_vol >> 8) & 0xFF);
      (void)USBD_CtlSendData(pdev, audio_ctl, 2U);
      return USBD_OK;
    }
    break;

  case AUDIO_REQ_GET_MIN:
  case AUDIO_REQ_GET_MAX:
  case AUDIO_REQ_GET_RES:
    if (cs == AUDIO_CS_VOLUME) {
      int16_t v = (req->bRequest == AUDIO_REQ_GET_MIN) ? AUDIO_VOL_MIN :
                  (req->bRequest == AUDIO_REQ_GET_MAX) ? AUDIO_VOL_MAX :
                                                         AUDIO_VOL_RES;
      audio_ctl[0] = (uint8_t)(v & 0xFF);
      audio_ctl[1] = (uint8_t)((v >> 8) & 0xFF);
      (void)USBD_CtlSendData(pdev, audio_ctl, 2U);
      return USBD_OK;
    }
    break;

  default:
    break;
  }
  USBD_CtlError(pdev, req);
  return USBD_FAIL;
}

static uint8_t Audio_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    return Audio_ClassReq(pdev, req);

  if ((req->bmRequest & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD) {
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
  }

  switch (req->bRequest) {
  case USB_REQ_GET_INTERFACE:
    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
      (void)USBD_CtlSendData(pdev, &audio_alt, 1U);
    } else {
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
    }
    break;

  case USB_REQ_SET_INTERFACE:
    if (pdev->dev_state == USBD_STATE_CONFIGURED) {
      if (LOBYTE(req->wIndex) == AUDIO_AS_IF) {
        audio_alt = LOBYTE(req->wValue);
        if (audio_alt == 1U) audio_usb_start();
        else                 audio_usb_stop();
      }
    } else {
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
  }
  return USBD_OK;
}

/* ------------------------------------------------------------------ */
/* Composite class callbacks.                                          */
/* ------------------------------------------------------------------ */
static uint8_t Composite_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)USBD_CDC.Init(pdev, cfgidx);

  (void)USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_MPS);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;
  audio_alt = 0U;
  audio_apply_gain();                      /* push current volume into audio.c */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, audio_rxbuf, AUDIO_MPS);
  return (uint8_t)USBD_OK;
}

static uint8_t Composite_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)USBD_CDC.DeInit(pdev, cfgidx);
  (void)USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;
  audio_usb_stop();
  return (uint8_t)USBD_OK;
}

static uint8_t Composite_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bmRequest & USB_REQ_RECIPIENT_MASK) {
  case USB_REQ_RECIPIENT_INTERFACE: {
    uint8_t ifn = LOBYTE(req->wIndex);
    if (ifn == AUDIO_AC_IF || ifn == AUDIO_AS_IF) return Audio_Setup(pdev, req);
    return USBD_CDC.Setup(pdev, req);
  }
  case USB_REQ_RECIPIENT_ENDPOINT: {
    uint8_t ep = LOBYTE(req->wIndex);
    if ((ep & 0x7FU) == (AUDIO_OUT_EP & 0x7FU)) return Audio_Setup(pdev, req);
    return USBD_CDC.Setup(pdev, req);
  }
  default:
    return USBD_CDC.Setup(pdev, req);
  }
}

static uint8_t Composite_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  if (audio_ctl_pending) {                 /* audio SET_CUR data arrived */
    audio_ctl_pending = 0U;
    if (audio_ctl_cs == AUDIO_CS_MUTE) {
      audio_mute = audio_ctl[0];
    } else if (audio_ctl_cs == AUDIO_CS_VOLUME) {
      int16_t v = (int16_t)((uint16_t)audio_ctl[0] |
                            ((uint16_t)audio_ctl[1] << 8));
      if (v < AUDIO_VOL_MIN) v = AUDIO_VOL_MIN;
      if (v > AUDIO_VOL_MAX) v = AUDIO_VOL_MAX;
      audio_vol = v;
    }
    audio_apply_gain();
    return (uint8_t)USBD_OK;
  }
  return USBD_CDC.EP0_RxReady(pdev);       /* CDC line coding */
}

static uint8_t Composite_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_CDC.DataIn(pdev, epnum);     /* only CDC has IN endpoints */
}

static uint8_t Composite_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == (AUDIO_OUT_EP & 0x7FU)) {
    uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);
    audio_usb_note_dout();
    audio_usb_write((const int16_t *)(const void *)audio_rxbuf, (int)(len / 2U));
    (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, audio_rxbuf, AUDIO_MPS);
    return (uint8_t)USBD_OK;
  }
  return USBD_CDC.DataOut(pdev, epnum);
}

static uint8_t Composite_IsoOUTIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  (void)epnum;
  audio_usb_note_isoinc();
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, audio_rxbuf, AUDIO_MPS);
  return (uint8_t)USBD_OK;
}

static uint8_t *Composite_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(Composite_CfgDesc);
  return Composite_CfgDesc;
}

static uint8_t *Composite_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(Composite_DeviceQualifier);
  return Composite_DeviceQualifier;
}

USBD_ClassTypeDef USBD_Composite =
{
  Composite_Init,
  Composite_DeInit,
  Composite_Setup,
  NULL,                         /* EP0_TxSent */
  Composite_EP0_RxReady,
  Composite_DataIn,
  Composite_DataOut,
  NULL,                         /* SOF */
  NULL,                         /* IsoINIncomplete */
  Composite_IsoOUTIncomplete,
  Composite_GetCfgDesc,         /* HS */
  Composite_GetCfgDesc,         /* FS */
  Composite_GetCfgDesc,         /* other speed */
  Composite_GetDeviceQualifierDesc,
};
