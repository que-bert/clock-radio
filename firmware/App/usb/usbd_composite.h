/* usbd_composite.h - CDC (console) + USB Audio Class speaker, one device.
 *
 * A single USBD class that owns all four interfaces. CDC traffic is delegated
 * to the stock ST USBD_CDC class (so the console is byte-for-byte unchanged);
 * the UAC1 speaker (interfaces 2-3, iso OUT endpoint) is handled here and fed
 * into the DAC via audio.c. Registered with plain USBD_RegisterClass, so the
 * core stays in its normal single-class configuration.
 */
#ifndef USBD_COMPOSITE_H
#define USBD_COMPOSITE_H

#include "usbd_def.h"

extern USBD_ClassTypeDef USBD_Composite;

#endif /* USBD_COMPOSITE_H */
