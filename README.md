# ELEC3304 Assessment 2：全浸式水翼船的高度与俯仰控制

被控对象是一条全浸式水翼小船（参数取自里斯本理工 SR02 太阳能船），目标是让船稳定飞在设定高度、船身保持水平，同时抵抗波浪和模型误差。

## 目前进度

| 部分 | 状态 | 在哪里看 |
|---|---|---|
| 非线性建模、配平、线性化（4 状态、2 输入、3 传感器） | 完成，已验证 | `model_derivation.pdf` 第 1–10 节 |
| 开环仿真（Lab 9 A2） | 完成 | `model_derivation.pdf` 第 11 节 |
| LQR 状态反馈（两个积分器、预滤波、抗积分饱和） | 完成 | `control_design.pdf` 第 2 节 |
| 卡尔曼滤波观测器（含偏差状态） | 完成 | `control_design.pdf` 第 3 节 |
| 50 Hz 数字实现（离散设计、计算延迟、采样率） | 完成 | `control_design.pdf` 第 4 节 |
| 不规则海况（JONSWAP 谱，三种海况） | 完成 | `control_design.pdf` 第 5 节 |
| 设计决策与权衡记录（D1–D12） | 持续更新 | `control_design.pdf` 第 6 节 |
| Simulink 完整闭环模型 | 完成，与 Matlab 仿真互相验证 | `matlab/hydrofoil_sim.slx`，图解见 `matlab/simulink_guide.html` |
| 回放动画 | 未开始 | — |

主要结果（50 Hz 数字控制器，非线性模型，含执行器限制和传感器噪声）：5 cm 高度阶跃约 2 s 稳定、无超调；5 cm 规则波中高度误差约 1.0 cm 均方根；不规则海况下按海况从 0.8 cm（有效波高 0.10 m）到约 4–5 cm（0.25 m）；航速 6–8 m/s、质量 195–235 kg 时稳态误差为 0。

注意：波浪模型曾有一个符号错误，已修正，所有受影响的结果都已重算（见 `control_design.pdf` 的 D12）。

## 文件结构

```
model_derivation.tex / .pdf   建模推导、参数、线性模型、开环分析与仿真（英文）
control_design.tex / .pdf     控制器、观测器、数字实现、设计决策记录（英文）
matlab/
  README.md                   每个脚本的作用和代码结构（中文，先看这个）
  simulink_guide.html         Simulink 模型逐模块图解（浏览器打开）
  hydrofoil_sim.slx           Simulink 模型，双击即可运行
  *.m                         建模、设计、仿真脚本
  figs/                       所有图
```

## 怎么运行

需要 Matlab（开发时用 R2024b）、Control System Toolbox 和 Simulink。在 Matlab 里把当前文件夹切到 `matlab/`，按顺序运行：

1. `run_openloop`：船本身的特性（图 1–5）
2. `run_lqr`：假设状态全知时的 LQR（图 7–10）
3. `run_observer`：卡尔曼滤波 + LQR（图 11–13）
4. `run_digital`：50 Hz 数字实现（图 15–17，需要几分钟）
5. `run_irregular`：不规则海况（图 18–19，需要几分钟）
6. `run_simulink`：Simulink 模型的验证（图 6、14）

或者直接双击 `matlab/hydrofoil_sim.slx`，点运行，看默认的闭环演示。

## 说明

- 参考文献（Belime 2020 硕士论文、Ren & Yang 2004、Liu et al. 2022、Faltinsen 2005）没有放进仓库，文档末尾有完整引用。
- 标了 `[est]` 的参数（弦长、浸深、传感器噪声、舵机速度等）是估算或假设值，见 `matlab/params.m`。
- 两份 PDF 由 `pdflatex` 编译，图片直接引用 `matlab/figs/`。
