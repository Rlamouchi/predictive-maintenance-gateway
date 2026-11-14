# Day log

One short entry per day: what I did, what I learned, what blocked me.

## Day 1
- Created the repository, README and CAN protocol v0.1.
## Day 2
- STM32CubeMX project for the STM32F407G-DISC1 with FreeRTOS (CMSIS-RTOS v2).
- 2 tasks: LedTask (blinks LD4 every 500 ms) and UartTask (prints a message every second).
- HAL timebase moved to TIM6 because FreeRTOS uses SysTick.
- printf redirected to SWV/ITM through the ST-Link (no USB-TTL cable needed).
