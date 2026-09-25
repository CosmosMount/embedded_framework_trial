#include "bridge_usb.h"
#include "app_usbx_device.h"
#include "main.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_descriptors.h"
#include "ux_dcd_stm32.h"

// Private USBX startup, CDC callbacks and PCD adaptation live together here.

static UX_SLAVE_CLASS_CDC_ACM* cdc_acm_handle = UX_NULL;

static void usb_cdc_activate(void* cdc_acm_instance)
{
    cdc_acm_handle = (UX_SLAVE_CLASS_CDC_ACM*)cdc_acm_instance;
}

static void usb_cdc_deactivate(void* cdc_acm_instance)
{
    (void)cdc_acm_instance;
    cdc_acm_handle = UX_NULL;
}

static void usb_cdc_parameter_change(void* cdc_acm_instance)
{
    (void)cdc_acm_instance;
}

UX_SLAVE_CLASS_CDC_ACM* usb_cdc_handle(void)
{
    return cdc_acm_handle;
}

// Registration numbers come from the generated CDC framework.
struct usb_cdc_registration
{
    uint8_t configuration;
    uint8_t interface_number;
};

static int usb_cdc_registration_from_framework(const uint8_t* framework, size_t length,
                                                struct usb_cdc_registration* out)
{
    uint8_t configuration = 0;
    if (framework == NULL || out == NULL)
    {
        return 0;
    }
    // Walk bLength-delimited descriptors, never native packed structs.
    for (size_t offset = 0; offset + 2U <= length;)
    {
        const uint8_t* descriptor = framework + offset;
        const size_t size = descriptor[0];
        if (size < 2U || size > length - offset)
        {
            return 0;
        }
        if (descriptor[1] == 2U && size >= 9U) // Configuration
        {
            configuration = descriptor[5];
        }
        else if (descriptor[1] == 4U && size >= 9U && // Interface
                 descriptor[3] == 0U && descriptor[5] == 2U &&
                 descriptor[6] == 2U && configuration != 0U) // CDC ACM
        {
            out->configuration = configuration;
            out->interface_number = descriptor[2];
            return 1;
        }
        offset += size;
    }
    return 0;
}

static UINT usb_controller_check(void)
{
    PCD_HandleTypeDef* pcd = pnx_usb_pcd();
    if (pcd == NULL || pcd->Instance == NULL)
    {
        return UX_ERROR;
    }
    // The current USBX pool/transfer path is configured for FIFO I/O, not PCD DMA.
    // Enabling DMA requires a separate cache-safe USBX pool/transfer contract.
    return pcd->Init.dma_enable == DISABLE ? UX_SUCCESS : UX_FUNCTION_NOT_SUPPORTED;
}

static int usb_controller_high_speed(void)
{
    return pnx_usb_pcd()->Init.speed == PCD_SPEED_HIGH;
}

static uint16_t fifo_words(uint16_t packet_size)
{
    const uint16_t words = (uint16_t)((packet_size + 3U) / 4U);
    return words < 16U ? 16U : words;
}

static UINT usb_controller_start(void)
{
    PCD_HandleTypeDef* pcd = pnx_usb_pcd();
    const int high_speed = usb_controller_high_speed();
    const uint8_t command_ep = USBD_CDCACM_EPINCMD_ADDR & 0x0FU;
    const uint8_t data_ep = USBD_CDCACM_EPIN_ADDR & 0x0FU;
    const uint8_t last_ep = command_ep > data_ep ? command_ep : data_ep;
    const uint16_t out_packet = high_speed ? USBD_CDCACM_EPOUT_HS_MPS : USBD_CDCACM_EPOUT_FS_MPS;
    const uint16_t data_packet = high_speed ? USBD_CDCACM_EPIN_HS_MPS : USBD_CDCACM_EPIN_FS_MPS;
    const uint16_t command_packet = high_speed ? USBD_CDCACM_EPINCMD_HS_MPS : USBD_CDCACM_EPINCMD_FS_MPS;
    // Room for two OUT packets plus setup/status bookkeeping. Sizes are words.
    const uint16_t rx_words = (uint16_t)(2U * fifo_words(out_packet) + 16U + 2U * pcd->Init.dev_endpoints);
    if (last_ep >= pcd->Init.dev_endpoints ||
        HAL_PCDEx_SetRxFiFo(pcd, rx_words < 128U ? 128U : rx_words) != HAL_OK)
    {
        return UX_ERROR;
    }
    // HAL computes offsets from all lower FIFOs: reserve 16 words for gaps too.
    for (uint8_t ep = 0; ep <= last_ep; ++ep)
    {
        uint16_t words = ep == 0U ? fifo_words(USBD_MAX_EP0_SIZE) : 16U;
        if (ep == command_ep) words = fifo_words(command_packet);
        if (ep == data_ep) words = fifo_words(data_packet);
        if (HAL_PCDEx_SetTxFiFo(pcd, ep, words) != HAL_OK)
        {
            return UX_ERROR;
        }
    }
    const UINT status = ux_dcd_stm32_initialize((ULONG)pcd->Instance, (ULONG)pcd);
    if (status != UX_SUCCESS)
    {
        return status;
    }
    return HAL_PCD_Start(pcd) == HAL_OK ? UX_SUCCESS : UX_ERROR;
}

// Older CubeMX USBX templates do not emit these H7-style application macros.
#ifndef USBX_DEVICE_MEMORY_STACK_SIZE
#define USBX_DEVICE_MEMORY_STACK_SIZE (7U * 1024U)
#endif
#ifndef UX_DEVICE_APP_THREAD_STACK_SIZE
#define UX_DEVICE_APP_THREAD_STACK_SIZE 1024U
#endif
#ifndef UX_DEVICE_APP_THREAD_PRIO
#define UX_DEVICE_APP_THREAD_PRIO 10U
#endif

static UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_parameter;
static TX_THREAD device_thread;

static void device_thread_entry(ULONG input)
{
    (void)input;
    if (usb_controller_start() != UX_SUCCESS)
    {
        // Match the board's existing startup failure policy; no recovery loop here.
        Error_Handler();
    }
}

// Link-selected replacement for CubeMX's application template, not its PCD code.
UINT MX_USBX_Device_Init(VOID* memory_ptr)
{
    TX_BYTE_POOL* pool = (TX_BYTE_POOL*)memory_ptr;
    UCHAR* memory;
    ULONG fs_length, hs_length = 0U, string_length, language_length;
    UCHAR* hs = UX_NULL;
    struct usb_cdc_registration registration;
    UINT status = usb_controller_check();
    if (status != UX_SUCCESS) return status;

    status = tx_byte_allocate(pool, (VOID**)&memory, USBX_DEVICE_MEMORY_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS) return status;
    status = ux_system_initialize(memory, USBX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0U);
    if (status != UX_SUCCESS) return status;

    // OTG_HS with the embedded FS PHY still receives only the FS framework.
    if (usb_controller_high_speed())
    {
        hs = USBD_Get_Device_Framework_Speed(USBD_HIGH_SPEED, &hs_length);
    }
    UCHAR* fs = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED, &fs_length);
    UCHAR* strings = USBD_Get_String_Framework(&string_length);
    UCHAR* languages = USBD_Get_Language_Id_Framework(&language_length);
    if (!usb_cdc_registration_from_framework(fs, fs_length, &registration))
    {
        return UX_ERROR;
    }
    status = ux_device_stack_initialize(hs, hs_length, fs, fs_length,
                                       strings, string_length, languages, language_length, UX_NULL);
    if (status != UX_SUCCESS) return status;

    cdc_parameter.ux_slave_class_cdc_acm_instance_activate = usb_cdc_activate;
    cdc_parameter.ux_slave_class_cdc_acm_instance_deactivate = usb_cdc_deactivate;
    cdc_parameter.ux_slave_class_cdc_acm_parameter_change = usb_cdc_parameter_change;
    status = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                           ux_device_class_cdc_acm_entry,
                                           registration.configuration, registration.interface_number,
                                           &cdc_parameter);
    if (status != UX_SUCCESS) return status;

    status = tx_byte_allocate(pool, (VOID**)&memory, UX_DEVICE_APP_THREAD_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS) return status;
    return tx_thread_create(&device_thread, "USBX device", device_thread_entry, 0U,
                            memory, UX_DEVICE_APP_THREAD_STACK_SIZE,
                            UX_DEVICE_APP_THREAD_PRIO, UX_DEVICE_APP_THREAD_PRIO,
                            TX_NO_TIME_SLICE, TX_AUTO_START);
}
