/*
 * G474 Main基板のpackage pinを用途別maskで全分類し、起動直後の安全IOを設定する。
 * M1ではUART/CAN/SPI/ADC/PWMを開始せず、通信pinを含む非使用pinはanalogにする。
 */
#include "board_io.h"

#include "stm32g474xx.h"

#include <stdint.h>

#define PIN_MASK(pin) (UINT32_C(1) << (pin))

/* STM32G474RE LQFP64でbondingされているGPIO。電源pinは含めない。 */
#define GPIOA_PACKAGE_MASK UINT32_C(0xFFFF)
#define GPIOB_PACKAGE_MASK UINT32_C(0xFFFF)
#define GPIOC_PACKAGE_MASK UINT32_C(0xFFFF)
#define GPIOD_PACKAGE_MASK PIN_MASK(2)
#define GPIOF_PACKAGE_MASK (PIN_MASK(0) | PIN_MASK(1))

/* PA4=IMU_CSはHighで非選択、PA15は既存安全値Low。PA13/14はSWDを保持する。 */
#define GPIOA_OUTPUT_MASK (PIN_MASK(4) | PIN_MASK(15))
#define GPIOA_OUTPUT_HIGH_MASK PIN_MASK(4)
#define GPIOA_INPUT_MASK PIN_MASK(10)
#define GPIOA_ANALOG_MASK UINT32_C(0x1BEF)
#define GPIOA_PRESERVE_MASK (PIN_MASK(13) | PIN_MASK(14))

/* PB2/PB10/PB14は用途未確定の既存出力、PB7はLED。すべてLowを安全値とする。 */
#define GPIOB_OUTPUT_MASK (PIN_MASK(2) | PIN_MASK(7) | PIN_MASK(10) | PIN_MASK(14))
#define GPIOB_OUTPUT_HIGH_MASK UINT32_C(0)
#define GPIOB_INPUT_MASK (PIN_MASK(5) | PIN_MASK(6) | PIN_MASK(8))
#define GPIOB_INPUT_PULLDOWN_MASK PIN_MASK(8) /* BOOT0兼用pinを実行開始後もLow側へ固定する。 */
#define GPIOB_ANALOG_MASK UINT32_C(0xBA1B)
#define GPIOB_PRESERVE_MASK UINT32_C(0)

/* PC12=buzzerをGPIO Lowに固定する。PC13/14はstatus LED、PC4はIMU interrupt。 */
#define GPIOC_OUTPUT_MASK (PIN_MASK(0) | PIN_MASK(5) | PIN_MASK(12) | PIN_MASK(13) | PIN_MASK(14))
#define GPIOC_OUTPUT_HIGH_MASK UINT32_C(0)
#define GPIOC_INPUT_MASK PIN_MASK(4)
#define GPIOC_ANALOG_MASK UINT32_C(0x8FCE)
#define GPIOC_PRESERVE_MASK UINT32_C(0)

#define GPIOD_OUTPUT_MASK UINT32_C(0)
#define GPIOD_OUTPUT_HIGH_MASK UINT32_C(0)
#define GPIOD_INPUT_MASK PIN_MASK(2)
#define GPIOD_ANALOG_MASK UINT32_C(0)
#define GPIOD_PRESERVE_MASK UINT32_C(0)

#define GPIOF_OUTPUT_MASK UINT32_C(0)
#define GPIOF_OUTPUT_HIGH_MASK UINT32_C(0)
#define GPIOF_INPUT_MASK UINT32_C(0)
#define GPIOF_ANALOG_MASK (PIN_MASK(0) | PIN_MASK(1))
#define GPIOF_PRESERVE_MASK UINT32_C(0)

#define ASSERT_DISJOINT(name, output, input, analog, preserve) \
  _Static_assert((((output) & (input)) | ((output) & (analog)) | ((output) & (preserve)) | ((input) & (analog)) | ((input) & (preserve)) | ((analog) & (preserve))) == 0U, \
    name " GPIO classification overlaps")

#define ASSERT_COMPLETE(name, package, output, input, analog, preserve) \
  _Static_assert(((output) | (input) | (analog) | (preserve)) == (package), name " GPIO classification incomplete")

ASSERT_DISJOINT("GPIOA", GPIOA_OUTPUT_MASK, GPIOA_INPUT_MASK, GPIOA_ANALOG_MASK, GPIOA_PRESERVE_MASK);
ASSERT_COMPLETE("GPIOA", GPIOA_PACKAGE_MASK, GPIOA_OUTPUT_MASK, GPIOA_INPUT_MASK, GPIOA_ANALOG_MASK, GPIOA_PRESERVE_MASK);
ASSERT_DISJOINT("GPIOB", GPIOB_OUTPUT_MASK, GPIOB_INPUT_MASK, GPIOB_ANALOG_MASK, GPIOB_PRESERVE_MASK);
ASSERT_COMPLETE("GPIOB", GPIOB_PACKAGE_MASK, GPIOB_OUTPUT_MASK, GPIOB_INPUT_MASK, GPIOB_ANALOG_MASK, GPIOB_PRESERVE_MASK);
ASSERT_DISJOINT("GPIOC", GPIOC_OUTPUT_MASK, GPIOC_INPUT_MASK, GPIOC_ANALOG_MASK, GPIOC_PRESERVE_MASK);
ASSERT_COMPLETE("GPIOC", GPIOC_PACKAGE_MASK, GPIOC_OUTPUT_MASK, GPIOC_INPUT_MASK, GPIOC_ANALOG_MASK, GPIOC_PRESERVE_MASK);
ASSERT_DISJOINT("GPIOD", GPIOD_OUTPUT_MASK, GPIOD_INPUT_MASK, GPIOD_ANALOG_MASK, GPIOD_PRESERVE_MASK);
ASSERT_COMPLETE("GPIOD", GPIOD_PACKAGE_MASK, GPIOD_OUTPUT_MASK, GPIOD_INPUT_MASK, GPIOD_ANALOG_MASK, GPIOD_PRESERVE_MASK);
ASSERT_DISJOINT("GPIOF", GPIOF_OUTPUT_MASK, GPIOF_INPUT_MASK, GPIOF_ANALOG_MASK, GPIOF_PRESERVE_MASK);
ASSERT_COMPLETE("GPIOF", GPIOF_PACKAGE_MASK, GPIOF_OUTPUT_MASK, GPIOF_INPUT_MASK, GPIOF_ANALOG_MASK, GPIOF_PRESERVE_MASK);

_Static_assert((GPIOA_OUTPUT_HIGH_MASK & ~GPIOA_OUTPUT_MASK) == 0U, "GPIOA high mask is not output");
_Static_assert((GPIOB_OUTPUT_HIGH_MASK & ~GPIOB_OUTPUT_MASK) == 0U, "GPIOB high mask is not output");
_Static_assert((GPIOC_OUTPUT_HIGH_MASK & ~GPIOC_OUTPUT_MASK) == 0U, "GPIOC high mask is not output");

static void set_pin_mode(GPIO_TypeDef * port, uint32_t pin, uint32_t mode)
{
  const uint32_t shift = pin * 2U;
  port->MODER = (port->MODER & ~(UINT32_C(3) << shift)) | (mode << shift);
}

static void set_pin_pull(GPIO_TypeDef * port, uint32_t pin, uint32_t pull)
{
  const uint32_t shift = pin * 2U;
  port->PUPDR = (port->PUPDR & ~(UINT32_C(3) << shift)) | (pull << shift);
}

static void configure_port(GPIO_TypeDef * port, uint32_t package_mask, uint32_t output_mask, uint32_t output_high_mask, uint32_t input_mask, uint32_t input_pulldown_mask,
  uint32_t analog_mask, uint32_t preserve_mask)
{
  const uint32_t output_low_mask = output_mask & ~output_high_mask;

  /* mode変更前にlatchへ安全値を書き、出力切替時のglitchを防ぐ。 */
  port->BSRR = (output_low_mask << 16U) | output_high_mask;
  port->OTYPER &= ~output_mask;

  for (uint32_t pin = 0; pin < 16U; pin++) {
    const uint32_t mask = PIN_MASK(pin);
    if ((package_mask & mask) == 0U || (preserve_mask & mask) != 0U) {
      continue;
    }

    port->OSPEEDR &= ~(UINT32_C(3) << (pin * 2U));
    set_pin_pull(port, pin, 0U);

    if ((output_mask & mask) != 0U) {
      set_pin_mode(port, pin, 1U); /* General purpose output */
    } else if ((input_mask & mask) != 0U) {
      if ((input_pulldown_mask & mask) != 0U) {
        set_pin_pull(port, pin, 2U);
      }
      set_pin_mode(port, pin, 0U); /* Input */
    } else if ((analog_mask & mask) != 0U) {
      set_pin_mode(port, pin, 3U); /* Analog */
    }
  }
}

void board_io_init_safe(void)
{
  /* 更新中はbuzzer PWMを禁止する。reset直後だけでなくdebug遷移時もTIM5を強制停止する。 */
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM5RST;
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM5RST;
  RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM5EN;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN | RCC_AHB2ENR_GPIOFEN;
  (void)RCC->AHB2ENR;

  configure_port(GPIOA, GPIOA_PACKAGE_MASK, GPIOA_OUTPUT_MASK, GPIOA_OUTPUT_HIGH_MASK, GPIOA_INPUT_MASK, 0U, GPIOA_ANALOG_MASK, GPIOA_PRESERVE_MASK);
  configure_port(GPIOB, GPIOB_PACKAGE_MASK, GPIOB_OUTPUT_MASK, GPIOB_OUTPUT_HIGH_MASK, GPIOB_INPUT_MASK, GPIOB_INPUT_PULLDOWN_MASK, GPIOB_ANALOG_MASK, GPIOB_PRESERVE_MASK);
  configure_port(GPIOC, GPIOC_PACKAGE_MASK, GPIOC_OUTPUT_MASK, GPIOC_OUTPUT_HIGH_MASK, GPIOC_INPUT_MASK, 0U, GPIOC_ANALOG_MASK, GPIOC_PRESERVE_MASK);
  configure_port(GPIOD, GPIOD_PACKAGE_MASK, GPIOD_OUTPUT_MASK, GPIOD_OUTPUT_HIGH_MASK, GPIOD_INPUT_MASK, 0U, GPIOD_ANALOG_MASK, GPIOD_PRESERVE_MASK);
  configure_port(GPIOF, GPIOF_PACKAGE_MASK, GPIOF_OUTPUT_MASK, GPIOF_OUTPUT_HIGH_MASK, GPIOF_INPUT_MASK, 0U, GPIOF_ANALOG_MASK, GPIOF_PRESERVE_MASK);
}

void board_status_set_validating(bool enabled)
{
  GPIOC->BSRR = enabled ? PIN_MASK(13) : (PIN_MASK(13) << 16U);
}

void board_status_set_invalid(bool enabled)
{
  GPIOC->BSRR = enabled ? PIN_MASK(14) : (PIN_MASK(14) << 16U);
}

void board_delay_cycles(unsigned int cycles)
{
  while (cycles-- > 0U) {
    __NOP();
  }
}
