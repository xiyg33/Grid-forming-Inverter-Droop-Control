# Grid-forming Inverter Droop Control (Simulink S-Function)

![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange.svg)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20MinGW64-blue.svg)
![Build](https://img.shields.io/badge/Build-mex%20INV__droopCtrl.cpp-informational.svg)

A grid-forming inverter droop-control implementation for MATLAB/Simulink using a C++ S-Function (MEX).

This project focuses on:

- GFM Inverter
- Presynchronization window control
- Droop-based voltage and frequency reference generation
- Voltage-current dual-loop control with dq decoupling and feedforward

## Author Statement

This project is designed and implemented by the author.

- Author: xiyg33
- Copyright (c) 2026 xiyg33

Contributions are welcome through issues and pull requests. Please keep attribution and license notices when redistributing or modifying this work.

## Project Structure

- INV_droopCtrl.cpp: Simulink S-Function entry and interface glue code
- inv_droop/config.hpp: configuration constants and output index layout
- inv_droop/math_utils.hpp: common math helpers (clamp, wrap, dead zone)
- inv_droop/transforms.hpp: abc-dq and dq-abc transforms
- inv_droop/control_blocks.hpp: reusable control blocks (PI, IIR, phase generator)
- inv_droop/controllers.hpp: controller modules (PLL, presync, droop, power)
- inv_droop/inverter_control.hpp: main inverter control pipeline

## Build and Usage

### Prerequisites

- MATLAB R2023b (or compatible)
- MinGW64 configured for mex
- understand the main circuit configuration

### Build MEX

In MATLAB command window, run in project root:

mex INV_droopCtrl.cpp

After successful compilation, a MEX binary (for example INV_droopCtrl.mexw64 on Windows) will be generated.

## I/O Overview

Inputs (6 ports):

1. Vgrid abc (3)
2. Vpcc abc (3)
3. IL abc (3)
4. IO abc (3)
5. Qset (1)
6. Pset (1)

Outputs (4 ports):

- Port0 (7): measured dq values and PLL angle
- Port1 (10): power path states, droop states, and voltage references
- Port2 (4): dual-loop current and dq voltage references
- Port3 (3): abc modulation references (Ua/Ub/Uc)

## Notes

- For reproducibility, keep control parameters in inv_droop/config.hpp and avoid ad-hoc edits inside interface glue code.

## License

This project is licensed under the MIT License.

See LICENSE for full text.
