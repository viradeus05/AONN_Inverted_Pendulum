# AONN Inverted Pendulum

Keil uVision firmware for the STM32F103C8 inverted pendulum controller.

## Build target

- Keil uVision project: `USER/WHEELTEC.uvprojx`
- MCU: `STM32F103C8` (Cortex-M3)
- Compiler: ARM Compiler 6 / ARMCLANG
- Flash image: `OBJ/WHEELTEC.hex`

Open `USER/WHEELTEC.uvprojx` in Keil uVision, install the `Keil.STM32F1xx_DFP` device pack if prompted, select the `MiniBalance` target, and build.

Generated files are intentionally excluded from version control. Keil will recreate the `OBJ` and `Listings` directories during a build.

## Repository scope

This repository contains the Keil-compatible firmware under `STM32_PID/Firmware` only. Alternative projects and non-Keil source trees are not included.

