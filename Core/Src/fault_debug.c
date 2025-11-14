#ifdef __cplusplus
extern "C" {
#endif
#include "fault_debug.h"

void HardFault_Debug(void)
    {
        __asm volatile(
            "tst lr, #4            \n"
            "ite eq                \n"
            "mrseq r0, msp         \n"
            "mrsne r0, psp         \n"
            "b HardFault_Handler_C \n"
        );
    }

    void HardFault_Handler_C(uint32_t *stack)
    {
        uint32_t r0  = stack[0];
        uint32_t r1  = stack[1];
        uint32_t r2  = stack[2];
        uint32_t r3  = stack[3];
        uint32_t r12 = stack[4];
        uint32_t lr  = stack[5];
        uint32_t pc  = stack[6];  // ← 崩溃地址
        uint32_t psr = stack[7];

        __BKPT(0);   // ★崩溃时停在这里★

        while (1);
    }

#ifdef __cplusplus
}
#endif
