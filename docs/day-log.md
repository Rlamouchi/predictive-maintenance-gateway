## Day 3
- Built the first Yocto image (poky scarthgap 5.0.21, core-image-minimal, qemux86-64).
- Booted it in QEMU and logged in as root. Root filesystem: 31 MB.
- Lessons: Ubuntu 24.04 user-namespace fix, rm_work to save disk, limit threads to avoid freezing the VM,
  VirtualBox NAT + virtio-net for full network speed, download first with --runall=fetch.# Day log

One short entry per day: what I did, what I learned, what blocked me.

## Day 1
- Created the repository, README and CAN protocol v0.1.
## Day 2
- STM32CubeMX project for the STM32F407G-DISC1 with FreeRTOS (CMSIS-RTOS v2).
- 2 tasks: LedTask (blinks LD4 every 500 ms) and UartTask (prints a message every second).
- HAL timebase moved to TIM6 because FreeRTOS uses SysTick.
- printf redirected to SWV/ITM through the ST-Link (no USB-TTL cable needed).
