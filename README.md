# 分数阶数全数字锁相环 (ADPLL)
## MATLAB 行为模型 → Verilog RTL 迁移

**状态**: 开发中 | **最后更新**: 2026年4月

---

## 📋 项目概述

本项目记录了**分数阶数ADPLL行为模型**从MATLAB到可生产级Verilog RTL的系统迁移过程。该设计通过**DTC斩波**、**LMS自适应校准**和**伪差分延时转换器**等先进技术，实现了低抖动和分数频率杂散抑制。

## 📁 文件结构及当前状态

```
/
├── phase_adder.v               开发中
├── LMS_Calib.v                 🔄 调试中 (INVKDTC崩溃)
├── digital_main.v              🔄 开发中
├── digital_top.v               🔄 开发中
├── DCO.v                       
├── DTC_analog_10b.v            ✅ 已集成 (行为模型)
├── DTC_decoder.v               ✅ 完成
├── DTC_dig.v                   ✅ 完成
├── TDC_AMS.v                   //应该与TDC重复了，后续看是否需要删除
├── Tracking.v                  ✅ 可用
├── FREF_generator.v            ✅ 完成
├── mmd.v                        ✅ 完成 (多模分频器)
├── pfd.v                        ✅ 完成 (鉴相频率检测器)
├── sd_mod.v                     ✅ 完成 (西格玛-三角调制)
├── chopper.v                    ✅ 完成 (信号交换器)
├── spi_master.v                ✅ 完成 (SPI接口)
├── SPI_slave.v                 ✅ 完成 (SPI接口)
├── tdc_core.v                  ✅ 完成 (TDC核心逻辑)
├── TDC.v                        ✅ 完成 (TDC包装)
│
├── top_sim_tb.v                🔄 测试台 (验证平台)
├── digital_top_tb.v            🔄 测试台 (单元测试)
├── spi_master_tb.v             🔄 测试台 (单元测试)
│
└── **文档及MATLAB模型**
    ├── Chopping_pro_td_v6.m         (MATLAB行为参考)
    ├── Chopping_pro_V4.md           (斩波技术及结果)
    ├── Chopping_DIFF推导.md         (差分模式数学推导)
    ├── CAL_OFT推导.md               (偏移DTC校准推导)
    │
    └── **参考论文**
        ├── A_Low_Jitter_and_Low_Spur_FractionalN_Digital_PLL_Based_on_a_DTC_Chopping_Technique.pdf
        └── A_DPD_DitherFree_DPLL_Based_on_a_Cascaded_Fractional_Divider_and_PseudoDifferential_DTCs_....pdf
```

---

## 📖 参考文献及推导

### 数学基础

#### DTC差分增益校准
来自 `Chopping_DIFF推导.md`:
```
φ_signed = acc1 - 0.5  [通过定点中MSB翻转实现]

在DIFF模式中:
  T_P = T_DTC(-(φ_signed/2) × GAIN)
  T_N = T_DTC(+(φ_signed/2) × GAIN)
  ΔT = T_N - T_P = T_DTC(φ_signed × GAIN)

MATLAB等效 (282–286行):
  DCW_P = -(phi_signed/2) * INVKDTC
  DCW_N = +(phi_signed/2) * INVKDTC
```

#### 偏移DTC校准
来自 `CAL_OFT推导.md`:
```
通过独立控制通路实现系统偏移消除
  OFFSET_DTC_CTRL ← 关联(PHE, 正信号)
  目标: 在零轴穿过处消除延时偏移
```

#### Legendre多项式 (奇数INL)
来自 `Chopping_pro_td_v6.m` 271–280行:
```
u_diff = (acc1 - 0.5) × 2  ∈ [-1, +1]
P3(u) = (5u³ - 3u) / 2     ← Legendre多项式, 3阶
校准: alpha_odd ← sign(PHE) × sign(P3) × step_size
校正: T_out = T_ideal + alpha_odd × P3(u)
```

### 同行评审参考
1. **MOLERI et al.** (2020): "Low Jitter and Low Spur Fractional-N Digital PLL Based on a DTC Chopping Technique"
   - 斩波架构主要参考
   - 覆盖环路稳定性、杂散抑制、抖动分析

2. **XU et al.** (2021): "DFD/Dither-Free DPLL Based on a Cascaded Fractional Divider and Pseudo-Differential DTCs"
   - 级联分频器拓扑
   - PD-DTC详细分析和失配建模

---


## 📊 信号流和关键公式

### 1. 分数累加器 (phase_adder.v)

**单端模式 (SE)** (`DIFF_EN=0`):
```
acc1 = {PHR_F(12位分数部分)}
phi_signed = acc1 - 0.5  (通过MSB翻转执行减法)
DCW_SE = (1 - acc1) × INVKDTC  [若CHOPPING=0]
       = acc1 × INVKDTC          [若CHOPPING=1]
```

**差分模式 (DIFF)** (`DIFF_EN=1`):
```
phi_signed = acc1 - 0.5
DCW_P = -(phi_signed / 2) × INVKDTC
DCW_N = +(phi_signed / 2) × INVKDTC

→ Verilog (完全无符号算术):
   diff_p_in = (1 - acc1/2) × INVKDTC = oneminus_half + QUARTER
   diff_n_in = (0 + acc1/2) × INVKDTC = acc1_half + QUARTER
   
其中 QUARTER ≈ 235 (10位DCW范围[0, 512]的中心点)
结果: DCW值在[117, 352]，X轴穿过点在235 ✅
```

**斩波效应**:
- 用2拍延迟的控制信号 (`Chopping_EN_Q2`) 翻转相位累加器符号
- 消除偶数阶INL和增益失配误差
- 激活时交换DIFF_P和DIFF_N通路

### 2. DTC控制字解码

每个DTC分支 (P和N) 接收一个10位控制字:
```
DTC_CTRL[9:0] = {4位二进制, 6位热计(上部)} + {4位热计(下部)}
    ↓
[DTC_decoder] → 63个上部电容开关 + 15个下部电容开关
    ↓
[DTC_analog_10b] → 可配置延时，含INL建模
```

**INL多项式** (MATLAB去趋势等效):
```
u = (ctrl_value - 512) / 512  ∈ [-1, 1]
INL_raw = A_INL_EVEN × u² + A_INL_ODD × u³
INL = detrend(INL_raw, 1)  ← 移除1阶趋势 (增益校准吸收)

参数范围:
  A_INL_EVEN: 10 ps (默认)
  A_INL_ODD:  60 ps (默认)
  失配度:    0-5% (P/N通路差异)
```

### 3. 校准环路 (LMS_Calib.v)

**符号-符号LMS算法**:
```
error = sign(PHE) × sign(输出信号)
update = (error != 0) ? mu × 方向 : 0
state = state + update
```

#### 3a. INVKDTC校准
- **输入**: 相位误差 (PHE, 12位有符号)
- **输出**: 反向增益常数 (INVKDTC, 12位无符号)
- **目标**: 归一化DTC斜率到1 ps/LSB


#### 3b. OFFSET_DTC校准
- **输入**: 零轴穿过处的相位误差
- **输出**: 偏移DTC控制 (10位无符号)
- **目的**: 消除系统性DTC延时偏移


#### 3c. Legendre多项式校准
- **输入**: 归一化相位的立方 (`u³`)
- **输出**: 奇数阶INL系数校正 (`ALPHA_ODD`, 16位有符号)
- **目的**: 通过Legendre多项式加权消除立方INL项



