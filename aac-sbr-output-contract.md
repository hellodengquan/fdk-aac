# AAC 解码输出格式契约：核心层 / SBR / 参数立体声 → 最终 PCM

本文基于仓库 `cpp/libAACdec`、`cpp/libSBRdec`、`cpp/libMpegTPDec`、`cpp/libPCMutils`、`cpp/libSACdec`、`cpp/libDRCdec` 的源码，分析：

- 音频配置（ASC / ADTS / LOAS-LATM / 带内 USAC 配置）如何决定**核心采样率**与**核心声道数**；
- SBR、参数立体声（PS）、MPEG Surround（MPS / MPS212）、UniDRC 下混、PCM 后处理（pcmDmx）如何把核心格式变换成**最终输出格式**；
- 中途配置变化（含隐式 SBR/PS 出现、PCE 变化、USAC DASH IPF 配置切换）时，`CStreamInfo` 中哪些字段在**哪一帧**更新；
- 调用方哪些缓冲/格式假设会因此失效，以及**何时必须重新读取输出格式**。

> 关键结论先说：`CStreamInfo::sampleRate / frameSize / numChannels` 是**逐帧导出的输出格式**，
> 每个 `aacDecoder_DecodeFrame()` 都可能改变；`aacSampleRate / aacSamplesPerFrame / aacNumChannels`
> 只是核心层（解码链中间态）格式，**绝不能当作最终 PCM 格式使用**。
> 头文件里 `aacNumChannels` 的注释已经明确写了 "CAUTION: This are not the final number of output channels!"。

---

## 1. 数据流与格式归属

```
传输层(TP)                   核心层(AAC/TCX/ACELP)        扩展层                     PCM 后处理
─────────────────────────────────────────────────────────────────────────────────────────────
ADTS/ADIF/LOAS/LATM/RAW  ──►  CAacDecoder_DecodeFrame  ──►  SBR(+PS)  ──►  MPS212/SAC  ──►  UniDRC ──► pcmDmx ──► limiter/交织 ──► pTimeData
   ASC/每帧 ADTS 头              输出 aacSamplesPerFrame     输出 SBR 帧率/PS 声道      输出树声道数          下混          min/max 声道
        │                         aacNumChannels=aacChannels
        └── 决定 aacSampleRate / aacSamplesPerFrame / extSamplingRate / AOT / flags
```

单帧内各阶段对 `self->streamInfo` 的写入顺序（`cpp/libAACdec/src/aacdecoder_lib.cpp`）：

| 序号 | 位置 | 动作 | 写入的格式字段 |
|---|---|---|---|
| ① | `aacdecoder_lib.cpp:1323` | 每帧开始 | `outputDelay = 0` |
| ② | `aacdecoder_lib.cpp:1344` | `CAacDecoder_DecodeFrame()` 核心解码 | `aacNumChannels`（`aacdecoder.cpp:3205`） |
| ③ | `aacdecoder_lib.cpp:1382-1387` | **把输出格式重置为核心格式** | `sampleRate = aacSampleRate`、`frameSize = aacSamplesPerFrame`、`numChannels = aacNumChannels` |
| ④ | `aacdecoder_lib.cpp:1503-1558` | `sbrDecoder_Apply()`（非 USAC SCFGI3） | `numChannels`、`sampleRate`（SBR 输出率），随后按比率重算 `frameSize`、`outputDelay`；PS 时置 `AC_PS_PRESENT` |
| ⑤ | `aacdecoder_lib.cpp:1565-1621` | `mpegSurroundDecoder_Apply()` | `numChannels`（=MPS 树输出声道数）、`frameSize`（通常不变）、`outputDelay` |
| ⑥ | `aacdecoder_lib.cpp:1625-1670` | `sbrDecoder_Apply()`（USAC 统一立体声，SCFGI3） | `sampleRate`（SBR 输出率）、按比率重算 `frameSize`；`AC_PS_PRESENT` 清零 |
| ⑦ | `aacdecoder_lib.cpp:1754-1758` | `FDK_drcDec_ApplyDownmix()`（USAC + UniDRC） | `numChannels` **可能再次改变** |
| ⑧ | `aacdecoder_lib.cpp:1822-1826` | `pcmDmx_ApplyFrame()` | `numChannels` 可上下混；`channelType/channelIndices` 可能被重写 |
| ⑨ | `aacdecoder_lib.cpp:1878-1879` | 每帧末 | `flags = self->flags[0]` |
| ⑩ | `aacdecoder_lib.cpp:2028-2041` | 返回前 | `bitRate`；一致性检查；输出无效时清零调用方缓冲 |

因此：**同一帧里 `numChannels` 最多被改写 5 次**（③ 核心 → ④ SBR/PS 或 ⑥ → ⑤ MPS → ⑦ UniDRC → ⑧ pcmDmx），
`frameSize` 最多 3 次（③ 核心 → ④/⑥ SBR 比率 → ⑤ MPS），`sampleRate` 最多 2 次（③ 核心 → ④/⑥ SBR）。

---

## 2. `CStreamInfo` 字段权威表

`cpp/libAACdec/include/aacdecoder_lib.h:795-888`

| 字段 | 语义 | 值来源 | 更新时机 | 是否每帧可变 |
|---|---|---|---|---|
| `sampleRate` | **最终** PCM 采样率 | ③ 核心率，④/⑥ 被 SBR 覆盖 | 每次成功解码帧 | **是**（隐式 SBR 出现时） |
| `frameSize` | **最终** PCM 每声道样本数 | ③ 核心帧长，④/⑥ 按 SBR 比率放大，⑤ MPS 一般不改，⑦ 不改 | 每次成功解码帧 | **是** |
| `numChannels` | **最终**输出声道数 | ③ 核心声道，④ PS 1→2，⑤ MPS 树，⑦ UniDRC 下混，⑧ pcmDmx 上/下混 | 每次成功解码帧 | **是** |
| `pChannelType` / `pChannelIndices` | 每声道类型/索引 | 核心 PCE/映射 → SBR 补 PS 声道 → MPS `SpatialDecChannelProperties` → pcmDmx `getChannelDescription` | 每次成功解码帧 | **是** |
| `aacSampleRate` | 核心采样率（ASC，已除 ELD downscale） | `CAacDecoder_Init`（`aacdecoder.cpp:2013`） | 仅配置（重新）初始化时 | 否（配置变化才变） |
| `aacSamplesPerFrame` | 核心帧长 | `aacdecoder.cpp:2015` | 仅配置初始化时 | 否 |
| `aacNumChannels` | 核心声道数（**非**最终声道） | `aacdecoder.cpp:3205`，每帧由比特流元素/PCE 统计 | **每帧** | **是**（PCE / 元素变化） |
| `extAot`, `extSamplingRate` | 扩展 AOT / 扩展采样率（ASC） | `aacdecoder.cpp:1907-1933` | 仅配置初始化 | 否 |
| `flags` | `AC_SBR_PRESENT` / `AC_PS_PRESENT` / `AC_MPS_PRESENT` 等 | 每帧合并 | 每帧 | **是** |
| `outputDelay` | 解码器附加延迟（**输出样本单位**） | 核心 + SBR + MPS + limiter 累加 | 每帧 | **是** |
| `bitRate` | 瞬时码率 | `nBits × sampleRate / frameSize`（`aacdecoder_lib.cpp:1119-1125`） | 每帧 | 是（用输出格式算） |

**要点**：

1. `aacSampleRate` / `aacSamplesPerFrame` / `extSamplingRate` 只在 `CAacDecoder_Init()` 的 `AC_CM_ALLOC_MEM` 分支写入，**与每帧的输出格式解耦**。
2. `aacNumChannels` 是**每帧**由本帧元素统计出来的（`aacdecoder.cpp:3205`），不是 ASC 常量；
   ADTS/MPEG-2 流里 PCE（`channelConfig == 0`）可以在**任意一帧**改变声道数与映射（见 §5.3）。
3. `outputDelay` 的 ③→④ 阶段会被 SBR 比率重新缩放（`aacdecoder_lib.cpp:1514-1539`），
   因此配置变化后不能沿用旧延迟值。
4. `flags` 与 `sbrEnabled` 的生命周期取决于传输层是否每帧重解析配置：
   - `CAacDecoder_Init()` 结束处会**无条件**执行 `self->sbrEnabled = sbrEnabled`（`aacdecoder.cpp:2467`）
     和 `CAacDecoder_AcceptFlags()`（`2465`）；其中局部 `sbrEnabled` 直接取自
     `asc->m_sbrPresentFlag`（`1917-1923`）。
   - **ADTS**：每个 ADTS 帧开始时都会走两遍 `cbUpdateConfig`（`tpdec_lib.cpp:887-943`），
     第二遍 `AC_CM_ALLOC_MEM` 会跑完整 `CAacDecoder_Init()`。ADTS 头不携带 SBR/PS 信息，
     所以**每帧开始时** `sbrEnabled` 和 `AC_SBR_PRESENT / AC_PS_PRESENT` 都被重置为 0；
     帧内若解析到 SBR/PS payload 再重新置 1（`aacdecoder.cpp:1001`、`aacdecoder_lib.cpp:1513/1551`）。
     由此推论：**ADTS 隐式 SBR 流中，某一帧若不含 SBR payload，该帧的输出会退回核心格式**
     （`aacdecoder_lib.cpp:1461` 的 `sbrEnabled` 为 0 → 不调用 SBR），下一帧再切回。
   - **ADIF**：头部只解析一次（`tpdec_lib.cpp:1476-1534`，以 `TPDEC_CONFIG_FOUND` 为条件），
     之后与 RAW/LOAS 一样不再逐帧覆盖。
   - **RAW / LOAS-LATM（无新 SMC）**：配置只给一次，`flags[0]` 与 `sbrEnabled` 不再被逐帧覆盖，
     一旦置位就粘滞到下一次配置变化。
   - 无论哪种传输，解码循环内对 `AC_SBR_PRESENT`/`AC_PS_PRESENT` 只有 `|=` / `&=~`
     （`aacdecoder_lib.cpp:1513,1551,1649,1664`），单帧来看表示"本帧确实跑了 SBR/PS"。

---

## 3. 核心采样率 → 输出采样率的对应关系

### 3.1 决定因素

- **核心率**：ASC 的 `m_samplingFrequency` / 每帧 ADTS 的 `sample_freq_index`，ELD 时再除 `downscaleFactor`；
- **输出率**：由 SBR 层的 `sampleRateOut` 决定，`sbrDecoder_Apply()` 末尾写回（`cpp/libSBRdec/src/sbrdecoder.cpp:1920`）；
- **SBR 输出率的来源**（`sbrDecoder_ResetElement()`，`sbrdecoder.cpp:310-327`）：

  | `sampleRateOut` 入参 | 行为 |
  |---|---|
  | `> 0`（ASC 显式 `extensionSamplingFrequency`） | 使用该值；`sampleRateIn == sampleRateOut` 时置 `SBRDEC_DOWNSAMPLE`（单速率 QMF 合成按 2:1 抽取，`synDownsampleFac = 2`） |
  | `== 0`（隐式 SBR，ASC 未给扩展率） | **假定双速率**：`sampleRateOut = sampleRateIn << 1`（`sbrdecoder.cpp:311-316`） |

- **帧长**：`frameSize` 由 `aacSampleRate` 与 `sampleRate` 的比率在 `aacdecoder_lib.cpp:1514-1546`（非 SCFGI3）与 `1650-1662`（SCFGI3）推导，支持 `1、2、4、8/3`（以及 ELD 下行 1/2、1/4 分支）。

### 3.2 典型对应表

| 场景 | ASC 信号 | `aacSampleRate`（核心） | `sampleRate`（输出） | `frameSize`（输出） | 备注 |
|---|---|---|---|---|---|
| AAC-LC，无 SBR | — | S | S | 1024 | `frameSize == aacSamplesPerFrame`（`aacdecoder_lib.cpp:1544-1545`） |
| AAC-LC + 显式 SBR 双速率 | extAOT=SBR, extRate=2S | S | 2S | 2048 | 比率 `<<1` 分支 |
| HE-AAC v2（显式 PS） | extAOT=SBR + `m_psPresentFlag=1`，核心 1ch | S | 2S | 2048 | 输出 2 声道（§4.1） |
| **隐式 SBR**（ADTS 头无 SBR 信息，SBR 只在 payload 里） | extRate 未给 | S | **2S（假定）** | **2048** | 直到首个带 SBR 的帧才生效，可能中途从 S/1024/1ch 切到 2S/2048 |
| 单速率 SBR（ASC 给出 `extRate == S`，如 ELD single-rate） | extRate == S | S | S | 核心帧长（512/480） | 置 `SBRDEC_DOWNSAMPLE`，QMF 合成按 2 抽取，比率 1 |
| ELD 带 downscale | `downscaledSamplingFrequency` | F/d | F/d；若 SBR 双速率则 `2·F/d` | `aacSamplesPerFrame/d`，双速率再 ×2 | `d ∈ {1,2,3,4}`（`aacdecoder.cpp:1849-1876`）；SBR 要求 d 为偶数（`1924-1928`） |
| USAC `coreSbrFrameLengthIndex=0` | 帧长 768，ratio 0 | S | S | 768 | 无 SBR |
| USAC `=1` | 帧长 1024，ratio 0 | S | S | 1024 | 无 SBR |
| USAC `=2` | ratio 8:3 | 3S/8 | S | 2048 | `frameSize == 768` → `×8/3` 分支（`aacdecoder_lib.cpp:1535-1539`） |
| USAC `=3` | ratio 2:1 | S/2 | S | 2048 | 比率 `<<1` |
| USAC `=4` | ratio 4:1 | S/4 | S | 4096 | 比率 `<<2` |

USAC 的比率映射在 ASC 解析阶段就已完成：`cpp/libMpegTPDec/src/tpdec_asc.cpp:2080-2118`
（`usacFrameLength = {768,1024,2048,2048,4096}`，`sbrRatioIndex = {0,0,2,3,1}`，
`m_samplingFrequency/m_samplesPerFrame` 被就地除以比率，`m_extensionSamplingFrequency` 保留输出率）。

> **注意**：`sampleRate` 与 `aacSampleRate` 是否相等**不能**用来判断"有没有 SBR 数据"。
> 单速率 SBR 时两者相等，但仍然跑 SBR 处理（只是 QMF 合成按 2 抽取）；此时 `AC_SBR_PRESENT` 会被置位
> （`aacdecoder_lib.cpp:1513`），而 `frameSize` 保持核心帧长。

---

## 4. 核心声道 → 输出声道的对应关系

### 4.1 参数立体声（PS）：单声道核心 → 立体声输出

判定条件在 `CAacDecoder_SyncQmfMode()`（`cpp/libAACdec/src/aacdecoder.cpp:255-260`）：

```c
self->psPossible =
    (CAN_DO_PS(aot) &&                    // AAC-LC / SBR / PS / ER-BSAC / DRM_AAC
     !PS_IS_EXPLICITLY_DISABLED(aot, flags) &&
     streamInfo.aacNumChannels == 1 &&    // 核心单声道
     !(flags & AC_MPS_PRESENT) &&         // 不与 MPS 同时启用
     qmfModeCurr == MODE_HQ);             // 必须 HQ QMF
```

`psPossible` 一旦为真，SBR 解码器**无条件输出两声道**：

- `sbrdecoder.cpp:1771-1795`：即使本帧没有 PS 数据，也会把 L 拷贝到 R，并 `*numOutChannels = 2`；
  注释明确："A decoder which is able to decode PS has to produce a stereo output even if no PS data is available."
- `aacdecoder_lib.cpp:1550-1557`：`AC_PS_PRESENT` 置位，为新增的第 2 声道填 `ACT_FRONT / index=1`。
- `sbrdecoder.cpp:1839-1841`：只有"单 SBR 元素且是 SCE"时才允许 PS。

因此，**核心单声道 + AAC-LC/SBR 家族 + HQ QMF + SBR 已启用**时，输出是 **2 声道**。
注意这是"SBR 被应用"为前提：`sbrEnabled == 0` 的纯 AAC-LC 帧根本不会调用 `sbrDecoder_Apply()`
（`aacdecoder_lib.cpp:1461`），因此仍按核心的 1 声道 / 1024 帧长输出；`psPossible` 此时虽为 1 也不起作用。
真正的切换点就是"隐式 SBR 第一次出现"的那一帧 → **1→2 声道、1024→2048 帧长、S→2S 采样率同时发生**。
RAW / LOAS 下 `sbrEnabled` 一旦置 1（`aacdecoder.cpp:1001`、`1189`）便保持到下一次配置变化，
即切换是单向的；**ADTS 下则不同**：每帧都会重跑 `CAacDecoder_Init()`，没有 SBR payload 的帧会把
`sbrEnabled` 重置回 0（`aacdecoder.cpp:1921`），于是输出格式会在 1ch/1024 与 2ch/2048 之间**来回跳**。
这正是 `aac-dec.c` 这类"只读一次格式"的调用方最容易崩坏的地方。

会让 `psPossible` 变假、从而**把输出从 2 声道退回 1 声道**的调用方设置：

| 设置 | 位置 | 后果 |
|---|---|---|
| `AAC_QMF_LOWPOWER = 1`（LP QMF） | `aacdecoder.cpp:198-208,255-260` | `qmfModeCurr = MODE_LP` → `psPossible = 0` → 单声道输出 |
| `AAC_PCM_MAX_OUTPUT_CHANNELS = 1` | `aacdecoder_lib.cpp:1484-1490` | 显式 `psPossible = 0`（要单声道就不做 PS） |
| 存在 MPS（`AC_MPS_PRESENT`） | `aacdecoder.cpp:259` | PS 与 MPS 互斥 |

> **陷阱**：`AC_PS_PRESENT` 的含义是"PS 处理已启用（psPossible）"，不是"本帧确实解出了 PS 数据"。
> 判断"是否立体声输出"请只看 `numChannels`。

### 4.2 MPS / MPS212

| 场景 | 核心声道 | 输出声道 | 代码 |
|---|---|---|---|
| USAC stereoConfigIndex=1（1 个核心声道 + MPS212） | 1 | 2 | `aacdecoder.cpp:1746-1767`、`285-295` |
| USAC stereoConfigIndex=2（2 个核心声道，残差 + MPS212） | 2 | 2 | 同上 |
| USAC stereoConfigIndex=3（统一立体声，MPS212 + SBR 末级上采样） | 1 或 2 | 2 | `aacdecoder_lib.cpp:1574-1578`、`1625-1670` |
| ELD v2（`useLdQmfTimeAlign` 且 SSC 可解析） | 1 | 2 | `aacdecoder.cpp:1950-1964`、`2107-2118` |
| GA（AAC-LC/HE-AAC）+ SAC 扩展 payload | 1 或 2 | SSC 树输出（`numOutputChannelsAT`，最多 6） | 见下 |

> **本仓库的可达性说明**：GA 路径的 MPS 由 `self->mpsEnableUser` 门控
> （`aacdecoder.cpp:940`、`aacdecoder_lib.cpp:1399-1403`），而 `aacDecoder_Open()` 把它初始化为 0
> （`aacdecoder_lib.cpp:1016`），公开的 `AACDEC_PARAM` 中也没有任何参数可以把它置 1。
> 因此经公共 API 的 GA 流即使带 SAC payload 也不会启用 MPS（`aacdecoder.cpp:942` 直接跳过解析，
> `AC_MPS_PRESENT` 不会被置位）。上表其余四行（USAC/ELD）不依赖 `mpsEnableUser`，是实际可达的。

`mpegSurroundDecoder_Apply()` 的输入上限是 2 声道（`sac_dec_lib.cpp:1550-1552`），
`frameSize` 由调用方传入并通常原样保留（`sac_dec_lib.cpp:1729-1739`），只有致命错误时才置 0。
失败时解码器会回退到"上次 MPS 成功输出的尺寸"并清零输出（`aacdecoder_lib.cpp:1606-1620`，
`mpsOutChannelsLast/mpsFrameSizeLast`，`aacdecoder.h:311-314`）——此时格式**不是**本帧真实格式。

MPS 也可能被**QMF 模式间接关掉**：当用户指定的 QMF 模式不被 MPS 支持时，
`aacdecoder.cpp:240-248` 会 `mpsEnableCurr = 0`，"disable MPS and playout the downmix" →
输出声道数会从"上混后的 2/6"掉回"核心下混声道"，**而码流没有任何变化**
（该分支只在 MPS 已启用时才会走到，即 USAC / ELD v2）。

### 4.3 UniDRC 下混与 pcmDmx

- `FDK_drcDec_ApplyDownmix()`（`aacdecoder_lib.cpp:1754-1758`）可在 USAC+UniDRC 时把 `numChannels` 减少
  （`cpp/libDRCdec/src/FDK_drcDecLib.cpp:840-899`，只做下混，不做上混）。
  注意：它**不改** `channelType/channelIndices`。
- `pcmDmx_ApplyFrame()`（`cpp/libPCMutils/src/pcmdmx_lib.cpp:2060-2141`）按 `AAC_PCM_MIN/MAX_OUTPUT_CHANNELS` 再改一次：

  | 参数 | 默认值 | 行为 |
  |---|---|---|
  | `numOutChannelsMax` | **6**（`pcmdmx_lib.cpp:1659`） | 编码 7.1（8ch）默认被**下混到 6ch** |
  | `numOutChannelsMin` | 0 | 设为 6/8 时做"补零上混"，新增声道 `ACT_NONE`（`pcmdmx_lib.cpp:2495-2515`） |
  | `applyProcessing` | 0（`DMX_DUAL_CHANNEL_MODE != STEREO` 时置 1，`pcmdmx_lib.cpp:1777`） | 为 0 且 min/max 不触发时直接 return，**不改声道数也不改通道类型表** |

  下混分支（`pcmdmx_lib.cpp:2186`）会用 `getChannelDescription()` 重写 `channelType/channelIndices`；
  上混分支把新增声道标为 `ACT_NONE`；**等声道分支（`pcmdmx_lib.cpp:2546`）不重写类型表**——
  这意味着 UniDRC 先下混（类型表未更新）后，pcmDmx 因数量已相等而不再修正，
  `pChannelType` 可能仍是下混前的描述。**只保证个数，不保证类型描述**。

---

## 5. 中途配置变化：哪些帧信息会更新

### 5.1 配置变化的三种触发源

| 触发源 | 入口 | 检测方式 | 典型后果 |
|---|---|---|---|
| 带外 ASC（`aacDecoder_ConfigRaw`） | `tpdec_lib.cpp:262-367` | 两遍解析：`AC_CM_DET_CFG_CHANGE` 比较 → `AC_CM_ALLOC_MEM` 应用 | 采样率/声道/AOT 改变；`cbFreeMem` 释放并 `CAacDecoder_DeInit` |
| 流内每帧头（ADTS） | `tpdec_lib.cpp:887-943` | 每帧 `adtsRead_DecodeHeader` → `cbUpdateConfig`（两遍） | 采样率索引/声道配置可逐帧变化；`sbrEnabled` 与 `flags[0]` 每帧被重置 |
| ADIF 头 | `tpdec_lib.cpp:1476-1534` | 仅在 `TPDEC_CONFIG_FOUND` 未置位时解析一次 | 之后同 RAW/LOAS，不再逐帧覆盖 |
| LOAS/LATM 带内 SMC、USAC 带内 Config | `tpdec_latm.cpp:169-…`、`tpdec_lib.cpp:369-560` | 与已存配置逐字节比较；USAC 支持 DASH IPF flush/build-up | 隐式/显式配置一致性检查；不一致 → `TRANSPORTDEC_PARSE_ERROR` |

`CAacDecoder_Init()`（`aacdecoder.cpp:1542-2479`）的配置变更判定字段：

- `aot`（1842）、`extSamplingRate`（1908）、`downscaleFactor`（1878）、
  **核心 `samplingRateInfo.samplingRate` 与 `aacSamplesPerFrame`**（1996-2021）、
  `ascChannels`（2024）、USAC 元素表/`stereoConfigIndex`（1736-1758）、ELD `useLdQmfTimeAlign`（1893-1905）。

> 注意：这里比较的是**核心**率与核心帧长。若 ASC 只改 `extensionSamplingFrequency`，也会触发 `ascChanged`（1908）；
> 而"隐式 SBR 从无到有"根本不经过这条路径（配置没变），却会改变输出格式（§5.2）。

### 5.2 隐式 SBR / PS 首次出现（无配置变化）

- `aacdecoder.cpp:2738-2762`：解码元素时若 `AC_SBR_PRESENT` 或 `sbrEnabled`，用
  `streamInfo.aacSampleRate / extSamplingRate / aacSamplesPerFrame` 初始化 SBR 元素；
- `aacdecoder.cpp:984-1006`：解析到 `EXT_SBR_DATA` 时初始化并 `sbrEnabled = 1`；
- `aacdecoder.cpp:1188-1190`：显式/隐式 SBR 解析成功后 `sbrEnabled = 1`。

结果是**在解到 SBR payload 的那一帧**，`aacdecoder_lib.cpp:1503-1558` 才第一次把
`sampleRate/frameSize/numChannels/outputDelay` 改成扩展格式。之前的帧仍是核心格式。
调用方若在第一帧就"记住格式"，之后必然错位。

在 ADTS 下这件事会**反复发生**：每帧的 `CAacDecoder_Init()` 都把 `sbrEnabled` 重置为
`asc->m_sbrPresentFlag`（=0），只有该帧真正解出 SBR payload 时才在帧内重新置 1
（`aacdecoder.cpp:1001`、`1189`）。所以一段隐式 SBR 的 ADTS 码流里，
**不含 SBR payload 的帧会短暂回到核心格式**（1ch/1024/S），含 SBR 的帧才是扩展格式（2ch/2048/2S）。
`sampleRate/frameSize/numChannels` 因此可能在相邻帧之间来回跳变——这是"每帧重读"最硬的理由。
（实践上隐式 SBR 编码器会在每帧都写 SBR payload，所以这种跳变少见；但契约上必须允许，
且损坏/丢帧时同样会出现。）

### 5.3 PCE 逐帧变化（MPEG-2 ADTS，`channelConfig == 0`）

- 每个 `raw_data_block` 都可能带 PCE：`aacdecoder.cpp:2858-2884`，
  解析后更新 `self->elements`、`self->chMapIndex`（2869）、`self->ascChannels[0] = pce->NumChannels`（2876）；
- 帧末 `aacNumChannels = aacChannels`（3205），并重排 `channelType/channelIndices`（3552-3571）；
- 若 PCE 要求声道数超过已分配数（`aacChannels > self->aacChannels`），该帧直接判错（`aacdecoder.cpp:3144-3153`），
  **不会**自动重新分配——只有缩小声道数能在运行中生效。

因此 `pChannelType/pChannelIndices` 与 `numChannels` 可以在**任意帧**变化，且 `channelConfig` 字段仍是 0。

### 5.4 单帧内配置变化时 `CStreamInfo` 的可见状态

| 时机 | `sampleRate/frameSize/numChannels` 的状态 | 调用方应做 |
|---|---|---|
| `aacDecoder_ConfigRaw()` 期间检测到变化 | `cbFreeMem` → `CAacDecoder_DeInit` 把 `aacSampleRate=0, sampleRate=0`（`aacdecoder.cpp:1429-1430`） | 视为"格式未知"，下一帧后重读 |
| 配置变化后、下一帧成功前 | 字段为 0 或上一次的旧值 | 不可用于分配缓冲 |
| `AAC_DEC_NOT_ENOUGH_BITS`（`aacdecoder_lib.cpp:1212-1214`） | 保持上一帧的值（**陈旧**），输出缓冲被清零 | 不读、不用 |
| SBR 处理返回 `SBRDEC_OUTPUT_BUFFER_TOO_SMALL`（`aacdecoder_lib.cpp:1559-1562`） | ③ 已写入核心格式，④ 未完成 → 看起来是核心格式 | 按错误处理，不要信任格式 |
| MPS 失败（`aacdecoder_lib.cpp:1606-1620`） | 回退为"上次 MPS 尺寸"，`sampleRate` 仍是 SBR 率 → **混合态** | 不信任，按 `AAC_DEC_DECODE_FRAME_ERROR` 处理 |
| USAC DASH IPF flush（`aacdecoder_lib.cpp:1895-1900`） | 中间态 `frameSize = 0`；返回前被一致性检查（`2031-2037`）判为 `AAC_DEC_UNKNOWN` | 该帧没有可消费 PCM；格式值不可信 |
| `AACDEC_CLRHIST` + USAC（`aacdecoder_lib.cpp:1186-1199`） | `numChannels = 0`（随后重建） | 重读 |
| `AACDEC_CONCEAL` / `AACDEC_FLUSH` | 声道数回退到 `aacChannelsPrev`、`channelType*Prev`、`sbrEnabledPrev`（`aacdecoder.cpp:3167-3202`） | 格式可能"回到上一成功帧"的形态 |
| 所有 `!IS_OUTPUT_VALID` 错误 | 输出缓冲被**整块**清零（`aacdecoder_lib.cpp:2039-2041`） | 丢弃 |

---

## 6. 调用方缓冲假设失效清单

| 假设 | 何时失效 | 依据 |
|---|---|---|
| "`numChannels` 等于编码声道数" | PS（1→2）、MPS（1/2→2/6）、UniDRC 下混、pcmDmx min/max（默认 7.1→6） | §4 |
| "`frameSize == aacSamplesPerFrame`" | 任何 SBR 放大（2×/4×/8⁄3）、USAC 8:3 与 4:1 | §3 |
| "缓冲按 ASC 的 `channels × samplesPerFrame` 分配" | 需要按**输出** `numChannels × frameSize`；HE-AACv2 相对核心是 **4 倍**（1×1024 → 2×2048） | `aacdecoder_lib.cpp:1931-1936` |
| "一帧 PCM 大小固定，可以预分配" | 隐式 SBR 首次出现、PS 首次出现、PCE 变化、USAC 配置切换；ADTS 隐式 SBR 下，缺 SBR payload 的帧会退回核心格式 | §5.2、§5.3 |
| "PCM 布局是平面的（planar）" | 最终 `pTimeData` **总是交织**：非 SBR/MPS 路径在 `aacdecoder_lib.cpp:2007-2016` 主动 `FDK_interleave`；SBR/MPS 路径本身输出交织（`aacdecoder_lib.cpp:1805-1808` 的 `interleaved` 标志）。内部 `outputInterleaved` 字段（`aacdecoder.h:197`）**从未被赋值**，不要读它 | `aacdecoder_lib.cpp:1805-1808`、`1997-2016` |
| "`sampleRate` 只由 ASC 决定" | 隐式 SBR 时由解码器假定 2×；单速率 SBR 时等于核心率 | `sbrdecoder.cpp:311-316` |
| "`outputDelay` 是固定值、只算一次" | 每帧重置（`aacdecoder_lib.cpp:1323`）、按 SBR 比率缩放（`1514-1539`）、MPS（`1601`）与 limiter（`1983`）再累加 | §2 |
| "`pChannelType` 描述最终声道" | UniDRC 下混后不更新；pcmDmx 等声道分支不重写 | §4.3 |
| "`AAC_DEC_NOT_ENOUGH_BITS` 时缓冲内容未动" | 返回前整块 `FDKmemclear(pTimeData, timeDataSize)` | `aacdecoder_lib.cpp:2039-2041` |

解码器内部上限可供参考：`pTimeData2` 内部缓冲为 `8 × 4096 × 2` 个 `PCM_DEC`（`cpp/libAACdec/src/aac_ram.cpp:176`），
即单帧最多 `8 × 4096` 个样本；调用方缓冲至少要覆盖该上限，或者按返回的 `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` 重分配。

---

## 7. 调用方何时**必须**重新读取输出格式

下面是规范性规则（"MUST"）。

1. **每调用一次 `aacDecoder_DecodeFrame()` 之后，只要本次返回的 `IS_OUTPUT_VALID(err)` 为真
   （`AAC_DEC_OK` 或 `aac_dec_decode_error_start..end`），就必须重新读取
   `sampleRate / frameSize / numChannels`。**
   理由：③~⑧ 全部是逐帧导出的字段；隐式 SBR、PS、PCE、UniDRC、pcmDmx 都可能在**没有配置变化**的情况下改它们。
2. **第一次成功解码帧之前，`sampleRate/frameSize/numChannels` 不可用**（初始化时被置 0，
   `aacdecoder.cpp:1255-1257`）；`aacDecoder_ConfigRaw()` 成功**不代表**格式已就绪。
3. **`aacDecoder_ConfigRaw()` / 检测到配置变化之后**：必须丢弃旧格式，至少解出一帧后再读。
4. **使用 `AACDEC_FLUSH` / `AACDEC_CONCEAL` / `AACDEC_INTR` / `AACDEC_CLRHIST` 之后**：重读；
   `CLRHIST`+USAC 还会短暂出现 `numChannels == 0`。
5. **解码返回 `AAC_DEC_NEED_TO_RESTART`**：`aacDecoder_Close()` + `aacDecoder_Open()` + 重新 `ConfigRaw()`，
   不能复用旧缓冲尺寸假设。
6. **调用 `aacDecoder_SetParam()` 改变了以下参数后**：重读（下一帧生效）。
   `AAC_PCM_MIN_OUTPUT_CHANNELS`、`AAC_PCM_MAX_OUTPUT_CHANNELS`、`AAC_PCM_DUAL_CHANNEL_OUTPUT_MODE`、
   `AAC_QMF_LOWPOWER`、`AAC_PCM_OUTPUT_CHANNEL_MAPPING`（`pChannelType` 顺序）。
7. **`flags` 中 `AC_SBR_PRESENT / AC_PS_PRESENT / AC_MPS_PRESENT` 为 1 的帧**：重读——
   这是"扩展层已介入"的最直接信号。但注意这些位在 RAW/LOAS 下会粘滞、在 ADTS 下会被每帧 ASC 覆盖
   （见 §2 要点 4），所以**不能**用"flag 为 0"推断本帧没有 SBR/PS；
   "SBR 是否启用"的权威内部状态是 `sbrEnabled`，它对外的可观察替身仍然是 `sampleRate` 与 `frameSize`。
8. **任何时候都不得把 `aacSampleRate/aacSamplesPerFrame/aacNumChannels` 当作 PCM 格式**，
   也不得用 `aacNumChannels` 判断输出声道数。
9. **`timeDataSize` 要以"样本数"（不是字节数）传入**，并且至少覆盖 `numChannels × frameSize`
   （`aacdecoder_lib.cpp:1931-1936`）；更稳妥的做法是按内部上限 `8 × 4096` 预分配，
   并在收到 `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` 后扩容。

### 推荐循环（伪代码）

```c
for (;;) {
    err = aacDecoder_Fill(dec, in, &size, &valid);
    if (err != AAC_DEC_OK) { /* 处理 */ }

    err = aacDecoder_DecodeFrame(dec, out_buf, out_capacity /* 样本数 */, flags);

    if (err == AAC_DEC_NOT_ENOUGH_BITS) continue;          /* 格式陈旧，勿用 */
    if (err == AAC_DEC_NEED_TO_RESTART) { reopen(); continue; }

    if (err == AAC_DEC_OUTPUT_BUFFER_TOO_SMALL) {
        /* 该帧的 access unit 已被消费（EndAccessUnit 已执行），本帧 PCM 丢失。
           按解码器内部上限扩容后从下一帧继续；不要读取当前 streamInfo 去配置 sink，
           因为失败点可能在 ④/⑤/⑧ 之前，格式仍是核心/中间态。 */
        grow(&out_buf, &out_capacity, MAX_FRAME_SAMPLES /* 8 × 4096 */);
        drop_frame();
        continue;
    }

    if (!IS_OUTPUT_VALID(err)) {                            /* 输出已被清零 */
        drop_frame(); continue;                             /* 含 IPF flush 的 frameSize==0 */
    }

    CStreamInfo *si = aacDecoder_GetStreamInfo(dec);        /* 每帧重读 */
    if (si->frameSize <= 0 || si->numChannels <= 0) continue;  /* 防御性兜底 */

    size_t need = (size_t)si->frameSize * si->numChannels;
    if (need > out_capacity) {                              /* 理论上不会发生 */
        grow(&out_buf, &out_capacity, need);
        continue;
    }
    if (need != last_need || si->sampleRate != last_rate ||
        si->numChannels != last_channels) {
        reconfigure_sink(si->sampleRate, si->numChannels, si->frameSize,
                         si->pChannelType, si->pChannelIndices);
        last_need = need; last_rate = si->sampleRate; last_channels = si->numChannels;
    }
    consume(out_buf, need, err);   /* AAC_DEC_DECODE_FRAME_ERROR 时为隐藏帧，但仍为有效 PCM */
}
```

---

## 8. 本仓库内调用方的现状（作为反例）

| 调用方 | 现有假设 | 风险 |
|---|---|---|
| `aac-dec.c:104-118` | 只在第一帧读取一次 `frameSize/numChannels/sampleRate`，并据此打开 WAV；`frame_size` 之后不重算 | 遇到隐式 SBR、PS、PCE 变化时 WAV 头与后续数据不匹配（写出的样本数与 WAV 头声道/采样率不符） |
| `aac-dec.c` / `test-encode-decode.c` | 固定 16 384 样本缓冲（8ch×2048），等于解码器内部上限的一半（内部为 8×4096，`aac_ram.cpp:176`） | 当前可达格式（USAC ≤2ch×4096）不会溢出，但一旦放宽 USAC 声道数或引入更高比率，就会 `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` |
| `test-encode-decode.c:111-116` | 每帧断言 `info->numChannels == channels`（输入 WAV 的声道数），并按 `numChannels*frameSize` 比较 PCM | 只有输入本身是 2 声道的 HE-AAC 才会通过（HE 编码把单声道升到立体声，见 `test-encode-decode.c:354-364` 的注释）。若用该断言去测"单声道 HE-AAC"或任何 PS 流，会把**正确的** 2 声道输出判为失败 |
| `tests/AacDecBenchmark/AacDecBenchmark.cpp:132` | `outputBuffer.resize(numChannels*frameSize)`（解码后才 resize） | 单帧安全，但没有重配置 sink 的逻辑；`getChannels/getFrameSize/getSampleRate` 的取值时机依赖调用顺序 |
| `cpp/fuzzer/aac_dec_fuzzer.cpp:29,118-124` | 固定 `2048 × 8` 样本；`DecodeFrame` 返回非 OK 就继续 | 未校验 `IS_OUTPUT_VALID`，未处理 `NEED_TO_RESTART` |
| `rust/framework/src/bin/decoder.rs:127-128,228,300-331` | `channels=2, sample_rate=44100, frame_size=1024` 全部**硬编码**，据此分配 `time_data`、写 WAV 头；flush 时用 `output_info.frame_size` 但 `channels` 从不更新 | 只能处理该固定格式的测试流；任何 SBR/PS/USAC 流都会缓冲不足或写出错位的 WAV |

---

## 9. 源码中值得复核的契约薄弱点

1. **`AAC_PCM_MAX_OUTPUT_CHANNELS` 的"默认"与文档**：头文件把 `-1/0` 定义为"禁用下混"，
   但解码器打开时 `pcmDmx` 的 `numOutChannelsMax` 已被初始化为 **6**（`pcmdmx_lib.cpp:1659`，由 `pcmDmx_Open` 调用），
   而 `aacDecoder_Open()` 不会把它改回 0。因此"什么都不设"时 **7.1（8ch）码流会被静默下混到 6 声道**；
   需要原样输出 8 声道的调用方必须显式 `SetParam(AAC_PCM_MAX_OUTPUT_CHANNELS, 8)`。
2. **`AC_PS_PRESENT` 语义**是"PS 可用并已启用"（`aacdecoder_lib.cpp:1550-1552` 依赖 `psPossible`），
   不能当作"本帧含 PS 数据"。
3. **UniDRC 下混不更新通道类型表**，随后 pcmDmx 在等声道分支也不再重写（`pcmdmx_lib.cpp:2546-2547` 注释
   "Don't need to change the channel description here"），`pChannelType` 可能与实际下混声道不符。
4. **`outputInterleaved` 字段（`aacdecoder.h:197`）从未被赋值**，纯属历史遗留；输出布局应视为"始终交织"。
5. **USAC SCFGI3 的 `frameSize` 回退分支**（`aacdecoder_lib.cpp:1658-1661`）在比率既非 8:3 也非 4:1 时
   默认按 2:1 处理，属于隐式假设；新增比率时容易漏改。
6. **`numChannels` 的注释说"i.e. the original channel configuration"**（`aacdecoder_lib.h:804-805`），
   但实现里它已经包含 pcmDmx 的上/下混结果（`pcmdmx_lib.cpp:2328`、`2543`），
   与"original channel configuration"的字面含义不一致——按"最终输出声道数"理解才对。
7. **`aacdecoder_lib.cpp:1480-1491`**：当 `AAC_PCM_MAX_OUTPUT_CHANNELS == 1` 且 SBR 启用时，
   解码器会直接关掉 `psPossible`。这是"输出声道需求反向影响解码路径"的隐式耦合：
   调用方只改一个 PCM 参数，就会改变 SBR 内部是否创建 PS 立体声输出。

---

## 10. 一页速查

```
核心格式（不可当最终格式）
  aacSampleRate / aacSamplesPerFrame / aacNumChannels     ← ASC + 逐帧元素统计

输出格式（每帧都可能变，MUST 每帧重读）
  sampleRate   = SBR ? SBR 输出率 : 核心率
  frameSize    = aacSamplesPerFrame × SBR 比率（1 / 2 / 4 / 8⁄3）
  numChannels  = 核心 → PS(1→2) → MPS(树) → UniDRC(下混) → pcmDmx(min/max, 默认 max 6)
  pChannelType / pChannelIndices ← 同上链路，可能滞后（见 §4.3）

必须重读的时机
  每帧 IS_OUTPUT_VALID 之后 | ConfigRaw 之后 | NEED_TO_RESTART 后重开
  FLUSH/CONCEAL/INTR/CLRHIST 之后 | SetParam(MIN/MAX/QMF/MAPPING) 之后
  AC_SBR_PRESENT / AC_PS_PRESENT / AC_MPS_PRESENT 为 1 的帧（ADTS 下它们逐帧重置）

不可信的帧
  NOT_ENOUGH_BITS（格式陈旧） | SBR 输出过小（格式为核心态） | MPS 失败（混合态）
  USAC DASH IPF flush（frameSize==0） | !IS_OUTPUT_VALID（缓冲已清零）

输出布局
  始终交织（L0 R0 L1 R1 ...），样本单位是 INT_PCM；timeDataSize 以样本数计
```

---

## 附：关键代码索引

| 主题 | 位置 |
|---|---|
| `CStreamInfo` 定义与字段注释 | `cpp/libAACdec/include/aacdecoder_lib.h:791-888` |
| 配置解析 / 变更检测 | `cpp/libAACdec/src/aacdecoder.cpp:1542-2479` |
| 核心解码与 `aacNumChannels` | `cpp/libAACdec/src/aacdecoder.cpp:2481-3210` |
| PCE 逐帧处理 | `cpp/libAACdec/src/aacdecoder.cpp:2858-2884`、`3240-3242`、`3552-3571` |
| PS 判定（`psPossible`） | `cpp/libAACdec/src/aacdecoder.cpp:187-262` |
| 每帧格式流水线 | `cpp/libAACdec/src/aacdecoder_lib.cpp:1344-2020` |
| SBR 输出率与 QMF 比率 | `cpp/libSBRdec/src/sbrdecoder.cpp:273-422`、`1800-1931` |
| PS 强制双声道 | `cpp/libSBRdec/src/sbrdecoder.cpp:1766-1795` |
| USAC 帧长/比率映射 | `cpp/libMpegTPDec/src/tpdec_asc.cpp:1672-1675`、`2080-2118` |
| 带外配置两遍解析 | `cpp/libMpegTPDec/src/tpdec_lib.cpp:262-367` |
| 带内 USAC 配置 | `cpp/libMpegTPDec/src/tpdec_lib.cpp:369-560`、`tpdec_latm.cpp:140-200` |
| ADTS 逐帧头 | `cpp/libMpegTPDec/src/tpdec_adts.cpp:166-340`、`tpdec_lib.cpp:887-968` |
| MPS 输出声道 | `cpp/libSACdec/src/sac_dec_lib.cpp:1524-1760` |
| UniDRC 下混 | `cpp/libDRCdec/src/FDK_drcDecLib.cpp:840-899` |
| pcmDmx 声道决策/默认值 | `cpp/libPCMutils/src/pcmdmx_lib.cpp:2060-2141`、`1654-1664` |
