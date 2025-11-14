#ifndef MY_MODULE_H
#define MY_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>        // 基本类型定义
#include "cmsis_gcc.h"     // __disable_irq() 等内核 intrinsic

    void HardFault_Debug(void);
    void HardFault_Handler_C(uint32_t *stack);

#ifdef __cplusplus
}
#endif

#endif  // MY_MODULE_H
