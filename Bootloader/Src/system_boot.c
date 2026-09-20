/*
 * G474 Mainブートローダーの最小SystemInitを提供し、C runtimeより前に安全IOを設定する。
 * M1ではreset既定のHSI 16 MHzを維持し、PLLや外部発振子を開始しない。
 */
#include "board_io.h"
#include "stm32g474xx.h"

#include <stdint.h>

uint32_t SystemCoreClock = UINT32_C(16000000);

void SystemInit(void)
{
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  SCB->CPACR |= (UINT32_C(3) << (10U * 2U)) | (UINT32_C(3) << (11U * 2U));
#endif

  SCB->VTOR = UINT32_C(0x08000000);
  board_io_init_safe();
}

void SystemCoreClockUpdate(void)
{
  SystemCoreClock = UINT32_C(16000000);
}

/* startupの__libc_init_arrayが要求する最小hook。ブートローダーでは処理不要。 */
void _init(void)
{
}

void _fini(void)
{
}
