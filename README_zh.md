# 构网型逆变器下垂控制（Simulink S-Function）

![许可证: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange.svg)
![平台](https://img.shields.io/badge/Platform-Windows%20%7C%20MinGW64-blue.svg)
![构建](https://img.shields.io/badge/Build-mex%20INV__droopCtrl.cpp-informational.svg)

🌍 [中文版本](README_zh.md) | [English Version](README.md)

一个用于 MATLAB/Simulink 的基于 C++ S-Function (MEX) 的构网型逆变器下垂控制实现。


本项目重点关注：

- 构网型逆变器
- 预同步窗口控制
- 基于下垂的电压和频率参考值生成
- 带有 dq 解耦和前馈的电压-电流双环控制

## 作者声明

本项目由作者设计与实现。

- 作者：xiyg33
- 版权所有 (c) 2026 xiyg33

欢迎通过提交 Issue 和 Pull Request 进行贡献。在重新分发或修改此作品时，请保留署名和许可证声明。

## 项目结构

- INV_droopCtrl.cpp：Simulink S-Function 入口和接口粘合代码
- inv_droop/config.hpp：配置常量和输出索引布局
- inv_droop/math_utils.hpp：常用数学辅助函数（钳位、环绕、死区）
- inv_droop/transforms.hpp：abc-dq 和 dq-abc 变换
- inv_droop/control_blocks.hpp：可复用的控制模块（PI、IIR、相位生成器）
- inv_droop/controllers.hpp：控制器模块（PLL、预同步、下垂、功率）
- inv_droop/inverter_control.hpp：逆变器主控制流程

## 构建与使用

### 先决条件

- MATLAB R2023b（或兼容版本）
- 为 mex 配置好的 MinGW64
- 理解主电路配置

### 构建 MEX

在 MATLAB 命令窗口中，于项目根目录运行：

mex INV_droopCtrl.cpp

成功编译后，将生成一个 MEX 二进制文件（例如在 Windows 上为 INV_droopCtrl.mexw64）。

## 输入/输出概述

输入（6 个端口）：

1. Vgrid abc (3)
2. Vpcc abc (3)
3. IL abc (3)
4. IO abc (3)
5. Qset (1)
6. Pset (1)

输出（4 个端口）：

- 端口0 (7)：测量的 dq 值和 PLL 角度
- 端口1 (10)：功率路径状态、下垂状态和电压参考值
- 端口2 (4)：双环电流/电压 dq 参考值
- 端口3 (3)：abc 调制参考值（Ua/Ub/Uc）

## 注意事项

- 为保证可复现性，请将控制参数保持在 inv_droop/config.hpp 中，避免在接口粘合代码中进行临时编辑。

## 许可证

本项目采用 MIT 许可证授权。

完整文本请参阅 LICENSE 文件。
