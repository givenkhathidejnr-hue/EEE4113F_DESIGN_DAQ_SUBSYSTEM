# EEE4113F Data Acquisition System

STM32L476RG firmware for a low-power, battery-operated optical data logger
(EEE4113F design project). The system wakes from deep sleep once an hour(ish),
takes a differential light measurement with ambient-light cancellation, logs
it to an SD card, and goes back to sleep — designed to run unattended for
extended periods on limited battery capacity.

## How it works

Each wake cycle runs through a small state machine (`g_daq.state` in
`daq.c`/`main.c`):

1. **INIT** — kick the watchdog, calibrate the ADC, confirm no power-domain
   fault is latched.
2. **POWER_SEQ** — enable the detection domain, wait for its Sallen-Key
   filter to settle.
3. **ACQUIRE** — a hardware timer (TIM2) drives a 30-pair on/off excitation
   cycle: LED off → settle → ADC sample, LED on → settle → ADC sample,
   repeated 30 times. Averaging pairs like this cancels ambient light and
   gains a few extra effective bits of resolution (ENOB) over a single
   sample.
4. **PROCESS** — mean/stddev over both buffers, differential
   (on − off), MCP9808 temperature compensation, CRC-32 over the record.
5. **LOG** — append to a RAM ring buffer (flushed to `DATA.CSV` on the SD
   card either when full or at the next opportunity).
6. **STOP** — enter STM32 Stop 2 mode (deepest RAM-retention sleep this part
   supports) until the RTC wakeup timer fires, then repeat.

Faults (a load-switch fault line, an ADC/DAC error, a bad detector reading,
an SD write failure, ...) route through a central `fault_handler()`: log the
fault (to SD, or a RAM fallback buffer if the SD write itself fails), retry
a few times, and fall back to sleeping until the next scheduled wake if the
fault won't clear. An independent watchdog (IWDG) guards against the MCU
hanging mid-cycle. `FaultManager`/`DaqContext`/etc. live in `.noinit` RAM
specifically so this fault history and cycle count survive a watchdog reset
instead of being wiped.

## Layout

```
Core/Inc, Core/Src   Application code + CubeMX-generated peripheral init
  daq.c/.h             The state machine and acquisition/logging engine
  INA219.h/.c          Current/power sensor driver (see Known gaps below)
  mcp9808.h            MCP9808 temperature sensor register map
  debug_strings.h      Fault/subsystem/state name tables for UART logging
Drivers/             STM32 HAL + CMSIS (vendor code, not modified)
FATFS/               FatFs middleware + SPI SD-card diskio driver
  Target/user_diskio_spi.c   Third-party (ChaN/kiwih) SD-over-SPI driver
Debug/               STM32CubeIDE build output (gitignored, IDE-regenerated)
Design_Project.ioc   CubeMX configuration
STM32L476RGTX_*.ld   Linker scripts
```

## Building

Import into STM32CubeIDE and build normally (Project → Build). `Debug/`'s
generated makefile embeds an absolute path from the machine it was last
built on, so it isn't meant to be run standalone outside the IDE — let
CubeIDE regenerate it. All source under `Core/`, `FATFS/`, and the drivers
was verified to compile and link cleanly (`-Wall -Wextra`, zero warnings)
with a plain `arm-none-eabi-gcc`/`ld` invocation using the flags CubeIDE's
own generated build uses, confirming the toolchain-specific pieces are fine
independent of the IDE project files.

## Known gaps

- **`INA219.c`/`.h`** (current/power sensor driver) is complete but
  currently unused.
- Everything here has been verified to compile, link, and (for the
  register-level logic) reason through carefully — treat it as a solid
  starting point for bench testing.
