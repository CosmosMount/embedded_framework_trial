#pragma once

// Private USBX/PCD bridge; applications use bsp_usb.hpp.
#include "usb_otg.h"

#ifdef __cplusplus
extern "C" {
#endif

// Generated from the selected IOC; CubeMX main owns PCD initialization.
PCD_HandleTypeDef* pnx_usb_pcd(void);

struct UX_SLAVE_CLASS_CDC_ACM_STRUCT;
typedef struct UX_SLAVE_CLASS_CDC_ACM_STRUCT UX_SLAVE_CLASS_CDC_ACM;

UX_SLAVE_CLASS_CDC_ACM* usb_cdc_handle(void);

#ifdef __cplusplus
}
#endif
