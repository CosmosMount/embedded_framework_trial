#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_exti_dispatch(uint16_t pin);

#ifdef __cplusplus
}
#endif
