// USB DFU "1200 baud touch" support for the STM32H753 on MARS boards.
//
// Lets the board be flashed over its USB-C port without holding BOOT0:
//   1. The host opens the USB CDC port at 1200 baud.
//   2. The SysTick hook below sees that, stores a magic word in RAM_D3 and
//      does a system reset.
//   3. On the next boot, before the Arduino core initializes anything, the
//      magic word is detected and we jump to the ST system bootloader in ROM,
//      which enumerates as a USB DFU device (0483:df11).
//   4. dfu-util flashes the new firmware and resets back into the app.
//
// The host side of this lives in scripts/dfu_touch.py.

#if defined(STM32H7xx) && defined(USBCON)

#include <Arduino.h>
#include "usbd_cdc.h"

// ST system bootloader entry for STM32H74x/H75x (AN2606).
static constexpr uint32_t SYSTEM_BOOTLOADER_ADDR = 0x1FF09800UL;

// Last word of RAM_D3 (SRAM4). The linker script does not place anything in
// RAM_D3, and SRAM contents are retained across a system reset.
static volatile uint32_t *const BOOT_MAGIC_ADDR =
    reinterpret_cast<volatile uint32_t *>(0x3800FFFCUL);
static constexpr uint32_t BOOT_MAGIC = 0xDF11B007UL;

static constexpr uint32_t TOUCH_BAUD = 1200;

// Globals owned by the core's USB CDC driver (usbd_cdc_if.c).
// USBSerial::baud() is hard-coded to 115200, so read the line coding directly.
extern "C" {
extern USBD_CDC_LineCodingTypeDef linecoding;
}

[[noreturn]] static void jumpToSystemBootloader() {
  __disable_irq();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  // The bootloader needs interrupts enabled (PRIMASK cleared) for USB.
  __enable_irq();

  const uint32_t *vectors = reinterpret_cast<const uint32_t *>(SYSTEM_BOOTLOADER_ADDR);
  __set_MSP(vectors[0]);
  __DSB();
  __ISB();
  reinterpret_cast<void (*)(void)>(vectors[1])();

  while (true) {
  }
}

// Runs before the core's premain() (priority 101), so the clocks, caches,
// HAL and USB are still in their reset state when we hand over to ROM.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wprio-ctor-dtor"
__attribute__((constructor(100))) static void checkBootloaderRequest() {
  if (*BOOT_MAGIC_ADDR == BOOT_MAGIC) {
    *BOOT_MAGIC_ADDR = 0;
    jumpToSystemBootloader();
  }
}
#pragma GCC diagnostic pop

// Called by the core every 1 ms from SysTick_Handler (weak default is a no-op).
// Running here means a reflash works even if loop() or setup() is blocked.
extern "C" void osSystickHandler(void) {
  // Trigger on the baud rate alone; don't depend on how the host OS drives DTR.
  if (linecoding.bitrate == TOUCH_BAUD) {
    *BOOT_MAGIC_ADDR = BOOT_MAGIC;
    // The core enables the D-cache, so the write above may only be in cache.
    // Flush it to SRAM, or it is lost on reset and we boot back into the app.
    SCB_CleanDCache_by_Addr(const_cast<uint32_t *>(BOOT_MAGIC_ADDR), 32);
    __DSB();
    NVIC_SystemReset();
  }
}

#endif
