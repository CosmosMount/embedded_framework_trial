#pragma once

#ifndef PNX_DMA_DEDICATED_SECTION
#define PNX_DMA_DEDICATED_SECTION 0
#endif

#ifndef PNX_DMA_CACHE_LINE_SIZE
#define PNX_DMA_CACHE_LINE_SIZE 4
#endif

#if defined(__GNUC__)
#if PNX_DMA_DEDICATED_SECTION
#define RAM_D1_BSS __attribute__((section(".ram_d1_bss")))
#define RAM_D2_BSS __attribute__((section(".ram_d2_bss")))
#define RAM_D3_BSS __attribute__((section(".ram_d3_bss")))
#define BSP_DMA_BUFFER __attribute__((section(".dma_buffer"), used, aligned(PNX_DMA_CACHE_LINE_SIZE)))
#else
#define RAM_D1_BSS
#define RAM_D2_BSS
#define RAM_D3_BSS
#define BSP_DMA_BUFFER __attribute__((used, aligned(PNX_DMA_CACHE_LINE_SIZE)))
#endif
#else
#define RAM_D1_BSS
#define RAM_D2_BSS
#define RAM_D3_BSS
#define BSP_DMA_BUFFER
#endif
