# 左右声道瞬态不同时的窗型同步契约

> 分析对象：FDK AAC（`cpp/libAACenc`, `cpp/libAACdec`, `cpp/libFDK`），分支 `task/101`。
> 主题：瞬态检测 → 窗序列切换 → 双声道窗型同步 → 可解码窗口关系的保持；
> 以及预回声控制、时频分辨率、联合立体声之间的取舍。

---

## 0. 标注约定（全文适用）

本报告把结论严格分成两类，**不要混读**：

| 标记 | 含义 | 判据 |
|---|---|---|
| **【约束·代码】** | 算法/实现的硬约束。可在源码中逐行指出，或用可执行探针复现。改动它会破坏码流或变换。 | `file:line` 可查；探针输出可复现 |
| **【约束·语法】** | 由 AAC 语法/解码器语义决定，代码只是执行它。 | 解码器断言/注释 + 语法元素表 |
| **【推断·可测】** | 由上述约束可推出的量化后果（如分辨率、比特数），不是听感结论。 | 可由公式或解析码流得出 |
| **【听感·未验证】** | 主观听感判断。**本报告没有做主观试听**，此类内容一律是"基于约束的预期"，不是结论。 | 未验证 |

**语义澄清**：本报告用两张表，**不是一个东西**：

- `chgWndSqLkAhd`（第 3 节，`block_switch.cpp:215-227`）：单声道状态机，把 `(lastattack, attack, P)` 映射到**该声道下一个窗型** `s`；
- `synchronizedBlockTypeTable`（第 4 节，`block_switch.cpp:411-424`）：双声道合并表，把 `(sL, sR)` 映射到**公共窗型**。

`chgWndSqLkAhd` 也是合并合法性证明的输入域：`S(P) = { chgWndSqLkAhd[la][at][P] : la, at ∈ {0,1} }`。两张表的角色不能互换。

凡是"听起来会怎样"的句子，只出现在第 8 节，且都带 `【听感·未验证】`。

---

## 1. 一句话结论

**【约束·代码】** FDK 的 AAC-LC 编码器把 `common_window` **硬编码为 1**（`cpp/libAACenc/src/psy_main.cpp:413` 的 `const INT commonWindow = 1;`，并在 `:1163` 写入码流）。
因此左右声道**在语法上不可能各自拥有独立的 `ics_info`**：`window_sequence`、`window_shape`、`max_sfb`、short-block 的 `scale_factor_grouping` 全部只出现一次，由两个声道共享。

所以"左声道进短窗、右声道保持长窗"这件事**编码器根本无法表达**。左右瞬态不同时的真实问题不是"怎么各用各的窗"，而是：

> 两个声道各自独立检测瞬态 → 得出各自的**建议窗型** → 用一张合并表把两者压成**一个公共窗型** → 再验证这个公共窗型与前一个公共窗型之间**斜率仍能拼合**（否则重叠相加不成立）。

---

## 2. 瞬态检测（每声道独立进行）

**【约束·代码】** `FDKaacEnc_BlockSwitching()`（`block_switch.cpp:229`）对每个声道单独调用（`psy_main.cpp:486`），左右声道各有独立的 `BLOCK_SWITCHING_CONTROL` 状态，**互不参考**。

### 2.1 分析窗划分

- LC：`nBlockSwitchWindows = 8`，每段 `granuleLength/8 = 128` 样本（`block_switch.cpp:177`, `:287`）。
- 每段累加两路能量（`FDKaacEnc_CalcWindowEnergy()`，`block_switch.cpp:359-409`）：
  - `windowNrg`：原始时域能量；
  - `windowNrgF`：先过二阶 IIR 高通（系数 `-0.5095/0.7548`，`block_switch.cpp:130-131`）后的能量。
- **判定只看高通后的能量**，即去除了低频稳态分量，使检测偏向"瞬态"而非"响"。

### 2.2 攻击判据

`block_switch.cpp:296-312`：

```c
enM1 = windowNrgF[0][nBlockSwitchWindows-1];          /* 上一帧最后一段 */
for (i = 0; i < 8; i++) {
  acc = 0.7*acc + 0.3*enM1;                           /* 泄漏积分参考 */
  if (0.1 * windowNrgF[1][i] > acc) {                 /* 当前段 > 10×参考 */
    attack = TRUE;
    attackIndex = i;                                  /* 覆盖写：取最靠后的一个 */
  }
  enM1 = windowNrgF[1][i];
}
if (enMax < minAttackNrg) attack = FALSE;             /* 绝对能量门限 */
```

**【约束·代码】** 三条固定参数，且都不可按声道、按内容自适应：

1. 相对门限 **10:1**（`invAttackRatio = 0.1f`，`block_switch.cpp:137-138`）；
2. 参考量是 **0.7/0.3 泄漏积分**（`accWindowNrgFac = 0.3f` / `oneMinusAccWindowNrgFac = 0.7f`，`block_switch.cpp:133-136`），时间常数约 3 个 128-样本段；
3. 绝对门限 `minAttackNrg`（`block_switch.cpp:143-145`），低于它不切短窗（避免静音段乱切）。

**【约束·代码】** `attackIndex` 在循环中被反复覆盖，最终记录的是**最靠后**满足条件的段，而不是最强的那一段。

### 2.3 跨帧延续

`block_switch.cpp:315-326`：若本帧无攻击、上帧有攻击、且上一帧最后一段能量 `> 10×` 本帧第 1 段能量、且上次攻击在 index 7，则强制 `attack=TRUE, attackIndex=0`。
**【约束·代码】** 这条规则专门处理"瞬态正好压在帧边界上"的情况，防止短窗序列提前结束、把瞬态尾巴留在长窗里。

---

## 3. 单一音轨的窗序列状态机

**【约束·代码】** 窗型不是自由选择的，而是由 `(lastattack, attack, 上一个窗型)` 三元组查表得到（`block_switch.cpp:215-227`）：

```c
static const INT chgWndSqLkAhd[2][2][N_BLOCKTYPES];
lastWindowSequence = chgWndSqLkAhd[lastattack][attack][lastWindowSequence];
```

LC 展开后的合法迁移（`chgWndSqLkAhd[lastattack][attack][P]`，列：从 LONG / 从 START / 从 SHORT / 从 STOP）：

| lastattack | attack | 从 LONG | 从 START | 从 SHORT | 从 STOP |
|---|---|---|---|---|---|
| 0 | 0 | LONG | SHORT | **STOP** | LONG |
| 0 | 1 | START | SHORT | SHORT | START |
| 1 | 0 | LONG | SHORT | SHORT | LONG |
| 1 | 1 | START | SHORT | SHORT | START |

（`lastattack` 是**上一帧**的检测结果：`block_switch.cpp:250` 先把当前 `attack` 存入 `lastattack`，之后才重算 `attack`；`P` 是上一帧的窗型。）

由此得到两条**必须遵守**的路径性质：

1. **进短窗必须先过 START**：`LONG --attack--> START`，而 `P=START` 时四种组合**全部**指向 SHORT，无法从 START 直接回到 LONG 或 STOP；
2. **出短窗必须先过 STOP**：`P=SHORT` 只可能停在 SHORT 或转 STOP，且转 STOP 的唯一条件是 `lastattack == 0 && attack == 0`（即当前帧与上一帧都没有攻击）。

**【推断·可测】** 一次瞬态的最短代价是 **4 帧**：`START + 1×SHORT + STOP + LONG`（`attack` 只持续一帧时可达），由 `probe16` 枚举 8 帧内全部攻击模式得到（找到的最短往返 = 4）。但跨帧延续规则（2.3 节）会把 `attack` 再延长一帧，所以实测通常为 **5 帧**（`START + 2×SHORT + STOP + LONG`）。无论哪种，其中**至少 2 帧（START、STOP）**本来是长窗帧。这个开销由状态机强制，不是编码器的优化选择。

> 与探针实测一致（`probe3` 场景 E，两声道同时受击，attack 连续两帧为真）：
> `f4 START → f5 SHORT → f6 SHORT → f7 STOP → f8 LONG`。
> 端到端编码结果也是这一形态（10.3 节）。

---

## 4. 双声道同步：合并表与分组

### 4.1 合并表

**【约束·代码】** `FDKaacEnc_SyncBlockSwitching()`（`block_switch.cpp:426`），在两个声道各自跑完检测后调用（`psy_main.cpp:498`）。合并分两步查表（`block_switch.cpp:435-437`）：

```c
patchType = synchronizedBlockTypeTable[LONG_WINDOW][L.lastWindowSequence];
patchType = synchronizedBlockTypeTable[patchType][R.lastWindowSequence];
```

`block_switch.cpp:411-424` 的表（行=中间结果，列=右声道；LC 只用到前 4 行 4 列）：

| L\R | LONG | START | SHORT | STOP |
|---|---|---|---|---|
| **LONG** | LONG | START | SHORT | STOP |
| **START** | START | START | SHORT | SHORT |
| **SHORT** | SHORT | SHORT | SHORT | SHORT |
| **STOP** | STOP | SHORT | SHORT | STOP |

（`LOWOV` 行/列在 LC 不可达——`LOWOV` 只由 `allowLookAhead=0` 的 `chgWndSq` 产生（`block_switch.cpp:204-213`），LC 走 `chgWndSqLkAhd`。相遇时返回 `-1`，被 `psy_main.cpp:503-505` 转成 `AAC_ENC_UNSUPPORTED_AOT`。这是防"LC/LD 混用"的护栏。）

### 4.2 合并的代数性质（探针实测，不是阅读推断）

用 `probe13` 对真实的 `FDKaacEnc_SyncBlockSwitching` 穷举 `{LONG,START,SHORT,STOP}⁴`：

```
LC merge on {LONG,START,SHORT,STOP}: commutative=1 idempotent=1 associative=1
```

**【约束·代码】** 即在 LC 可达域上，合并是一个**交换、幂等、结合**的半格运算。这些性质不是巧合，根源在于表的结构：

- `synchronizedBlockTypeTable[LONG_WINDOW][X] == X` 对全部 X 成立（`block_switch.cpp:412` 的 LONG 行是恒等行）。因此第一步 `T[LONG][L]` **返回 `L` 本身**，两步查表退化为一层 `T[L][R]`。
- `T` 的 LC 4×4 子表**对称**（`probe17` 逐格比对：非对称格 0 个），于是 `T[L][R] == T[R][L]`，合并天然可交换。

**【约束·代码】** 由此：

- 合并结果与"先算左还是先算右"无关；
- 合并是**保守的**：只要任一声道想进短窗体制，结果就偏短窗。

> 需要提醒的是：现代码里第一步查表**恒等于恒等映射**，所以它目前是冗余的。但两层结构传达了一个意图——"先算一个中间档位，再并入第二声道"。若将来把 `synchronizedBlockTypeTable[LONG_WINDOW]` 那一行改成非恒等（例如引入加权的中间档），交换性就不再自动成立，`probe13` 的结论也会失效。这一点在改动该表时必须复验。

具体语义（逐格验证）：

| 公共上一窗型 P | 单声道可能建议 | 合并结果 |
|---|---|---|
| LONG | {LONG, START} | 任一为 START 则 **START** |
| START | {SHORT} | **SHORT** |
| SHORT | {STOP, SHORT} | 任一为 SHORT 则 **SHORT** |
| STOP | {LONG, START} | 任一为 START 则 **START** |

**关键性质：受击声道的选择永远不会被另一个声道覆盖掉。** 因为受击声道在每种 P 下建议的都是该集合里"最偏短窗"的那个元素，而合并恰好取并集里最偏短窗的元素。代价只由**未受击声道**承担。

### 4.3 分组（grouping）同步

**【约束·代码】** 窗型合并后，若 `patchType == SHORT_WINDOW`，8 个短窗的分组也必须统一（`block_switch.cpp:461-532`）：

- **两个声道原本都是 SHORT** → 取 `maxWindowNrg` 较大的那一方的分组（`block_switch.cpp:486-505`）；
- **只有一个原本是 SHORT** → 直接用它（`block_switch.cpp:506-522`）；
- 若 `patchType != SHORT` → 强制 `noOfGroups = 1`（`block_switch.cpp:469-480`）。

**【约束·代码】** `maxWindowNrg` 的来源值得注意（`block_switch.cpp:275-280`）：

```c
if (attack /* 这是进入本次调用时的值，即上一帧的检测结果 */)
  maxWindowNrg = GetWindowEnergy(windowNrg[0], lastAttackIndex);
else
  maxWindowNrg = 0;
```

它取的是**上一帧**攻击段的能量，而 `attack/attackIndex` 在函数后段才被重算。所以分组归属依据的是"上一帧谁更响"，不是"本帧谁触发了合并"。

**【约束·代码】** 平局规则：比较用严格 `>`（`block_switch.cpp:488`），**两者相等时右声道胜出**。探针 `probe12` 实测：`maxWindowNrg` 都为 0 时，左 `(1,1,3,3)` / 右 `(3,3,1,1)` → 结果取右的 `(3,3,1,1)`。

### 4.4 一个不可达分支（实现细节，非缺陷）

`block_switch.cpp:523-531` 有一条注释为 "syncing a start and stop window ..." 的分支，把分组设成 `2 groups of 4`。

**【约束·代码】** 它**不可达**：`:463-466` 读出的 `windowSequenceLeftOld/RightOld` 在 `:445-446` 已经把左右都写成 `patchType` 之后才读取，因此二者恒等于 `patchType`；进入 `else` 分支（`:483`）的前提是 `patchType == SHORT`，于是"两个都是 SHORT"的第一分支（`:486-487`）必然命中，后面两条 `else if`（`:506`, `:515`）及最后的 `else`（`:523`）都是死代码。探针 `probe7` 对全部 25 种输入组合穷举，该分支命中 **0 次**。

> 同理，`windowSequenceRightOld` 在 `:465` 的读取发生在类型覆盖之后，语义上永远是 `patchType`；这三条分支可整体删除而不改变行为。

---

## 5. 核心问题：左右瞬态不同、处于长短窗过渡时，**可解码窗口关系**如何保持

这是本次分析的主问题，答案分三层。

### 5.1 语法层：左右不可能各写各的 `ics_info`

**【约束·代码】** FDK 把 `commonWindow` 定为编译期常量 1：

- `psy_main.cpp:413`：`const INT commonWindow = 1;`
- `psy_main.cpp:1163`：`psyOutElement->commonWindow = commonWindow;`
- `bitenc.cpp:1006-1011`：把该值写入 `common_window` 位。

**【约束·语法】** 在 `common_window == 1` 的 CPE 语法里，`ics_info()` 只出现一次：
`cpp/libFDK/src/FDK_tools_rom.cpp:6583-6600` 的 `el_aac_cpe1[]` 为
`{ics_info, ms, global_gain, section_data, ..., spectral_data, next_channel, ...}` —— 只有一个 `ics_info`，且位于 `next_channel` 之前。

**【约束·语法】** 解码器据此把 ch0 的 `icsInfo` 整体拷给 ch1（`cpp/libAACdec/src/channel.cpp:512-516`）：

```c
if (numberOfChannels == 2 &&
    pAacDecoderChannelInfo[0]->pDynData->RawDataInfo.CommonWindow) {
  pAacDecoderChannelInfo[1]->icsInfo = pAacDecoderChannelInfo[0]->icsInfo;
}
```

**结论（语法层）**：左右声道在码流中**不存在**窗型冲突的可能。同步不是在"修正一个非法码流"，而是在**决定那个唯一的公共值取什么**。

### 5.2 变换层：合法窗序列 ⟺ 左斜率等于上一帧右斜率

**【约束·代码】** 解码器由 `window_sequence` 反推左右窗斜率长度（`cpp/libAACdec/src/block.cpp:1035-1064`）：

| window_sequence | 左斜率 `fl` | 右斜率 `fr` |
|---|---|---|
| `BLOCK_LONG` | N | N |
| `BLOCK_START` | N | N/8 |
| `BLOCK_SHORT` | N/8 | N/8 |
| `BLOCK_STOP` | N/8 | N |
| `BLOCK_LONG`(LD, shape=2) | N | N/4 |

其中 N 为帧长（LC 默认 1024）。最后一行只在 LD/ELD 下出现：`shape==2` 由解码器在 `AC_LD` 时把 `window_shape` 位改写成 2 得到（`channelinfo.cpp:145-149`），`getWindow2Nr()` 再返回 `3N/4`（`block.cpp:969-978`），故 `fr = N - 3N/4 = N/4`。**LC 的 `window_shape` 只有 0/1，`getWindow2Nr` 恒返回 0，`fr` 恒等于 `fl`。**

**【约束·代码】** 解码器的左窗**不是**由当前帧的 `window_shape` 决定的，而是直接续用上一帧存下来的右窗（`cpp/libFDK/src/mdct.cpp:512` 的 `pWindow = hMdct->prev_wrs;`，在 `:710-714` 写回）。同理编码器侧 `wls = hMdct->prev_wrs; fl = hMdct->prev_fr;`（`cpp/libFDK/src/mdct.cpp:175-177`）。

> **这一点直接解释了为什么 `blockType2windowShape[LC]` 里 START/SHORT 用 SINE 不会破坏 TDAC**：SINE 只作用于这两类窗型的**右斜率**；左斜率继承自上一帧（LONG/STOP 用 KBD），与当前帧的 shape 字段无关。

由此得到**精确 TDAC 的充分条件**：

> **`fl(当前帧) == fr(上一帧)`**（长度与系数都相等）。满足它时重叠区两侧窗函数互补，长时无混叠完全抵消。

**【约束·代码】** 注意措辞：这是**充分**条件，不是"可解码"的充要条件。见 5.4 节——`fl != prev_fr` 时解码器仍能输出声音，只是 TDAC 不再精确。

**【约束·代码】** 编码器侧同样按此建模：`FDKaacEnc_Transform_Real()` 里 `fr` 由窗型决定（`cpp/libAACenc/src/transform.cpp:141-153`），上一帧的 `fr` 通过 `hMdct->prev_fr` 传给下一帧当 `fl`（`mdct.cpp:175-177`, `:710-714`）。

**这是第 4 节合并表能保证可解码性的依据。** 把四种窗型的 `(fl, fr)` 代入，得到"斜率可衔接"集合 `L(P) = { next | fl(next) == fr(P) }`，再与状态机实际可产生的集合 `S(P)` 比较（`probe14`）：

| P | `fr(P)` | 斜率可衔接 `L(P)` | 状态机可达 `S(P)`（`chgWndSqLkAhd`） | `S ⊆ L`？ |
|---|---|---|---|---|
| LONG | N | {LONG, START} | {LONG, START} | ✓ 相等 |
| START | N/8 | {SHORT, STOP} | {SHORT} | ✓ **真子集** |
| SHORT | N/8 | {SHORT, STOP} | {STOP, SHORT} | ✓ 相等 |
| STOP | N | {LONG, START} | {LONG, START} | ✓ 相等 |

**【约束·代码】** `P = START` 时 `S(P)` 是 `L(P)` 的**真子集**：斜率上 `START → STOP` 也合法，但状态机禁止（`chgWndSqLkAhd[·][·][START]` 四格全为 SHORT）。即状态机比"斜率可衔接"**更保守**，这是实现的选择，不是语法要求。

**【约束·代码】** 因此正确的说法是：`S(P) ⊆ L(P)` 对四种 P 均成立。这意味着合并表只需保证输出落在 `L(P)` 内；而由于输入来自 `S(P)` 且合并后仍在 `S(P)` 内（4.2 节的表），输出必然也在 `L(P)` 内。

> 注意不能反过来声称"合并表 = 斜率合法性"。合并表是**保守闭包**：它对 `P=START` 从不产生 `STOP`，尽管 `STOP` 在斜率上合法。这个保守性来自窗序列的语义（START 之后必须进短窗），而不是 TDAC 的数学要求。

**【约束·代码】** 更一般地，对真实函数的穷举（`probe11`）证实：拿 `chgWndSqLkAhd` 能产生的所有建议组合去喂合并表，输出集合与上表一致，**非法结果 0 个**：

```
P     succ-set of channel L/R after chgWndSqLkAhd   merged-outcomes (all 2x2 combos)
LONG  {LONG,START,LONG,START}  -> {LONG,START}
START {SHORT,SHORT,SHORT,SHORT} -> {SHORT}
SHORT {STOP,SHORT,SHORT,SHORT}  -> {STOP,SHORT}
STOP  {LONG,START,LONG,START}   -> {LONG,START}
illegal merged outcomes: 0
```

因此可以给出**契约**：

> **窗型同步契约（Window-Sync Contract）**
> 1. 左右声道共享唯一的 `window_sequence`（`common_window=1` 强制）；
> 2. 该公共窗型由 `T[T[LONG][sL]][sR]` 决定，`sL/sR` 是两声道各自的独立检测建议；
> 3. **前提不变量**：`sL, sR ∈ S(P)`，`S(P)` 由 `chgWndSqLkAhd` 的 `P` 列给出，且 `S(P) ⊆ L(P) = { n | fl(n) == fr(P) }`；
> 4. 上一帧的窗型同样是公共的，故上一帧的 `fr` 对两声道相同；
> 5. ⇒ 合并结果落在 `S(P)` 内（4.2 节的闭包性质），因而也落在 `L(P)` 内，两个声道同时满足 `fl == fr(prev)`，重叠区窗函数互补，TDAC 关系不被破坏。

**关键点**：这个契约的成立**不依赖**"哪个声道触发了合并"。即使是右声道受击、左声道安静，合并出来的公共窗型依然对左声道是合法的——因为合法性与"受击者是谁"无关，只与"上一个公共窗型的右斜率"有关。

**【约束·代码】契约的成立依赖第 3 条的前提，不能单独引用合并表。** `probe18` 把 LC 表的**全部 16 格**都按 `fl(merged) == fr(P_L) && fl(merged) == fr(P_R)` 检查，结果 **12 格不满足**：

```
T[LONG ][START]=START : fl(merged)=8, fr(prev L)=8, fr(prev R)=1  <-- INVALID
T[LONG ][SHORT]=SHORT : fl(merged)=1, fr(prev L)=8, fr(prev R)=1  <-- INVALID
T[START][LONG ]=START : fl(merged)=8, fr(prev L)=1, fr(prev R)=8  <-- INVALID
...
```

也就是说，合并表对**任意**输入并不安全，它只在"输入来自 `S(P)`"的前提下安全。若有人绕过 `chgWndSqLkAhd` 直接给 `lastWindowSequence` 赋一个非法值，`FDKaacEnc_SyncBlockSwitching()` 不会报错，会照常写出一个斜率不衔接的窗型。

> 现有代码里这个不变量由两点保证：`FDKaacEnc_BlockSwitching()` 只通过 `chgWndSqLkAhd` 写 `lastWindowSequence`（`block_switch.cpp:328-339`，LFE 除外），以及 `FDKaacEnc_InitBlockSwitching()` 的初值是 `LONG`（`block_switch.cpp:185`）。`probe11` 正是沿着这条可达路径穷举的；`probe18` 则是对**域外**输入的负面对照。

### 5.3 窗形与 `max_sfb`：合并后再派生，天然一致

**【约束·代码】** 窗形由合并后的窗型反查（`block_switch.cpp:449-454`）：

```c
windowShape = blockType2windowShape[allowShortFrames][lastWindowSequence];
```

LC 行（`block_switch.cpp:122-124`）为 `{KBD, SINE, SINE, KBD, WRONG}`（LONG/START/SHORT/STOP/LOWOV），即 LONG 与 STOP 用 KBD，START 与 SHORT 用 SINE。

因为窗形是**在合并之后**派生的，它天然对两声道相同，与解码器 `stereo.cpp:1101-1102` / `:1213-1214` 的断言（MS / IS 要求左右 `WindowShape` 相等）自洽。

**【约束·代码】** `max_sfb` 也共享。编码器在立体声处理前先把两声道取最大（`psy_main.cpp:1164-1166`）：

```c
if (psyOutElement->commonWindow)
  maxSfbPerGroup[0] = maxSfbPerGroup[1] = fixMax(maxSfbPerGroup[0], maxSfbPerGroup[1]);
```

这是共享 `ics_info` 的必然结果：`el_aac_cpe1` 里只有 ch0 的 `ics_info` 会被写出（`bitenc.cpp:1018` 在 `ch=0` 时执行），ch1 的 `max_sfb` 只能继承。若不先取最大，ch1 的高频会被截断。

### 5.4 解码器的容错（不是契约的一部分）

**【约束·代码】** 万一遇到不合规码流（`fl != prev_fr`），解码器有兜底：`imlt_block()` 检测到 `prev_fr != fl` 时调用 `imdct_adapt_parameters()`（`mdct.cpp:484-486`, `:372-419`），通过调整重叠长度来对齐。

这保证"能解"，但**不保证 TDAC**。所以它是健壮性措施，不能当作"窗型可以随便配"的依据。

### 5.5 一个不对称后果：单侧持续攻击会无限延长短窗体制

这是"左右瞬态不同"最需要写进契约的一条，也是**只靠读合并表看不出来**的。

**【约束·代码】** 把 4.2 节的语义和 3 节的状态机合起来看：当 `P = SHORT` 时，单声道建议只有两种可能（由 `chgWndSqLkAhd[·][·][SHORT]` 给出）——`SHORT`（`attack` 为真，或 `lastattack` 为真）或 `STOP`（仅当 `lastattack == 0 && attack == 0`）。而合并表 `T[SHORT][STOP] = T[STOP][SHORT] = SHORT`。

> 只要**任一声道**在 SHORT 期间继续保持 `attack`（哪怕另一声道一直想退出），公共窗型就**永远停在 SHORT**，直到该声道自己也停止攻击。

`probe15` 用真实函数逐帧验证这一格的行为（每帧把两声道分别置于 "建议 SHORT" 与 "建议 STOP"，再调用 `FDKaacEnc_SyncBlockSwitching`）：

```
f0: L wants SHORT, R wants STOP -> merged SHORT
f1: L wants SHORT, R wants STOP -> merged SHORT
...
f4: L wants SHORT, R wants STOP -> merged SHORT
```

（该探针固定输入以隔离单格行为，不复现完整状态机；完整状态机路径由 `probe16` 的穷举给出。）

**【推断·可测】** 这不是 bug，而是"公共窗型必须兼顾两声道"的直接推论。但它有两个可量化的后果：

1. **短窗时长由攻击最久的那个声道决定。** 右声道（安静侧）承受的短窗帧数 = 左声道的攻击持续帧数，而不是它自己需要的时间。
2. **STOP 帧会被推迟，而不是被跳过。** 一旦攻击停止，状态机仍需 `SHORT（最后帧）→ STOP → LONG`（3 节），所以安静侧的"恢复长窗"至少还要再等 2 帧。

**【约束·代码】** 与 2.3 节的跨帧延续规则叠加后，单次瞬态实际占用的短窗帧数可能比"攻击持续时长"多一帧——这条规则本意是防止瞬态尾巴落在长窗里，但它同时也会推迟公共窗型回退。

**【推断·可测】** 上述 1、2 是算法后果的可测量描述（帧数、哪一侧承担）。它们在听感上是否显著（例如安静侧是否听得出发闷）属于第 8 节 L2/L3 的范畴，本报告未验证。

---

## 6. 取舍一：预回声控制 vs 时频分辨率 vs 比特

本节把"算法上确定"与"听感上待验证"分开写。

### 6.1 算法上确定的部分

**【约束·代码】** 短窗路径把一帧拆成 8 次变换（`psy_main.cpp:522-527`）：

```c
windowLength[ch] = psyConf->granuleLength / TRANS_FAC;   /* 1024/8 = 128 */
nWindows[ch]    = TRANS_FAC;                             /* 8 */
maxSfb[ch]      = MAX_SFB_SHORT;
```

**【推断·可测】** 量化噪声在时域的分布被限制在变换块内。以 44.1 kHz、N=1024 为例：

| 量 | 长窗 | 短窗 | 比值 |
|---|---|---|---|
| 时间跨度（噪声可扩散范围） | 1024 样本 = 23.22 ms | 128 样本 = 2.90 ms | 1/8 |
| 频率分辨率（bin 宽） | 44100/1024 ≈ 43.1 Hz | 44100/128 ≈ 344.5 Hz | 8× 变粗 |
| `max_sfb` 字段 | 6 bit（`bitenc.cpp:208`） | 4 bit（`bitenc.cpp:218`） | — |
| 额外语法 | — | `scale_factor_grouping` 7 bit（`bitenc.cpp:221`） | — |
| 每帧独立 scale factor 上限 | 1 组 | 最多 4 组（`MAX_NO_OF_GROUPS`，`psy_const.h:141`） | — |

**【约束·代码】** 分组上限为 4（`psy_const.h:141`），所以 8 个短窗**不可能各自带一组 scale factor**；`suggestedGroupingTable`（`block_switch.cpp:191-199`）的每行都是 4 个长度、和为 8。

**【约束·代码】** 短窗还会改变共享工具的行为：

- **PNS**：低复杂度路径下短窗直接跳过检测（`aacenc_pns.cpp:196-199`）；另一条路径在 `JUST_LONG_WINDOW` 且非 LONG 时跳过（`:201-205`）。立体声 PNS 预处理也只在 ch0 非 SHORT 时执行（`psy_main.cpp:1167-1170`）。
- **TNS**：`FDKaacEnc_TnsSync()` 在"一声道短窗、另一声道非短窗"时**直接返回不同步**（`aacenc_tns.cpp:969-974`）。由于本编码器窗型恒同步，这条分支实际不可达，属防御性代码。
- **比特储备**：只要任一声道是 SHORT，整对就按短窗估算比特因子（`adj_thr.cpp:2733-2742`）。

**【约束·代码】** 临界带的直接后果：一个声道受击，另一个声道**也被拖进短窗体制**，且要多付 START/STOP 两帧。探针 `probe19` 实测（只有右声道在第 4 帧第 1 段受击、左声道全程安静；`attack`/`lastattack` 为该帧 `FDKaacEnc_BlockSwitching()` 返回后的值）：

```
f4 pre L=LONG  R=START | L.attack=0 L.lastattack=0 | R.attack=1 R.lastattack=0 -> post L=START R=START
f5 pre L=SHORT R=SHORT | L.attack=0 L.lastattack=0 | R.attack=0 R.lastattack=1 -> post L=SHORT R=SHORT
f6 pre L=STOP  R=STOP  | L.attack=0 L.lastattack=0 | R.attack=0 R.lastattack=0 -> post L=STOP  R=STOP
f7 pre L=LONG  R=LONG  | L.attack=0 L.lastattack=0 | R.attack=0 R.lastattack=0 -> post L=LONG  R=LONG
```

左声道的 `attack` 与 `lastattack` 全程为 0（从未受击），但它的窗型序列仍与右声道完全一致地被拖成 `LONG→START→SHORT→STOP→LONG`。

**【约束·代码】** 实测码流也印证了这一点。用真实编码器对"只有左声道有瞬态"和"两声道都有瞬态"两个文件编码，ADTS 解析出的公共窗序列**完全相同**：

```
frame 21: START        shape=0 (SINE)  cw=1
frame 22: EIGHT_SHORT  shape=0 (SINE)  cw=1  grouping=0b1101100
frame 23: EIGHT_SHORT  shape=0 (SINE)  cw=1  grouping=0b0110110
frame 24: STOP         shape=1 (KBD)   cw=1
frame 25: LONG         shape=1 (KBD)   cw=1
```

（`left.aac` 与 `both.aac` 在这四帧上的窗型/窗形/分组逐位相同；解码后只有左声道在该处出现 +16 dB 的能量跳变，右声道保持 1 dB 不变。）

### 6.2 算法上的取舍结构（可陈述，但无主观结论）

**【推断·可测】** 取舍是**结构性的**，不是调参可以回避的：

1. 要压预回声 → 必须进短窗；
2. 进短窗 → 频率分辨率固定损失 8×，且至少多付 2 帧的 START/STOP；
3. 窗型共享 → 损失由**未受击声道**承担，且无法用"该声道不需要短窗"来豁免；
4. 分组上限 4 → 短窗内的时频折衷也不自由，最细只能到 4 组。

### 6.3 不能从代码/约束推出的部分

以下问题本报告**没有**回答，也不能仅凭源码回答：

- 在具体素材上，8× 频率分辨率损失和 8× 时间分辨率收益，哪个占主导；
- START/STOP 这两帧对非受击声道的实际可闻影响；
- 分组由"上一帧更响的声道"决定，在快速交替受击时是否会造成分组抖动。

这些属于第 7 节的听感范畴。

---

## 7. 取舍二：联合立体声（MS / IS）的窗型耦合

### 7.1 算法上确定的部分

**【约束·语法】** MS 与 IS 只能在 `common_window == 1` 的 CPE 中使用：

- 解码器 MS 路径的断言与注释：`cpp/libAACdec/src/stereo.cpp:1097-1102`
  > `M/S joint channel coding can only be used if common_window is 1.`
- 解码器 IS 路径的注释与断言：`stereo.cpp:1207-1214`
  > `...only in the right channel of a channel_pair_element() having a common ics_info() (common_window == 1).`

**【约束·代码】** 编码器把 IS 再额外挂在 `commonWindow` 上（`psy_main.cpp:1187`, `:1213`），而 `commonWindow` 恒为 1，所以该门形同虚设。

**【约束·代码】** 由于窗型和分组都被同步，MS/IS 的**时间粒度**也被绑定：

- 长窗时：每帧一次 MS 判决，按 sfb；
- 短窗时：按 `noOfGroups × sfbCnt_short` 组织（`psy_main.cpp:1210-1214`, `:1221-1225`），且 `noOfGroups/sfbCnt` 来自同步后的分组。

**【推断·可测】** 于是联合立体声的时频分辨率**不可能优于**窗型同步的结果。具体地：

- 分组越粗（如 `3,3,1,1`），MS 在时间上的判决点越少；
- 分组由 `maxWindowNrg` 较大的一方决定（4.3 节），所以**未受击声道的 MS 粒度可能被受击声道的能量分布牵着走**；
- `maxSfbPerGroup` 被强制取两声道最大（`psy_main.cpp:1164-1166`），意味着一个声道的宽带内容会让另一个声道多传 scale factor。

**【约束·代码】** 一个容易忽略的耦合：`msMask` 的索引跨度是 `noOfGroups * sfbCnt_short`（`psy_main.cpp:1210-1212`），其大小随分组变化。分组同步若不一致，MS 掩码与谱线分组就会错位。这也是分组必须同步、而不能"各用各的"的实现原因。

### 7.2 算法层可下的结论

- **窗型同步不是为了省比特，而是正确性前提**：MS/IS 的语法本身就要求 `common_window=1`。
- **联合立体声的时间分辨率是窗型同步的因变量**，不是独立可调的旋钮。
- **代价不对称**：受击声道拿到它需要的短窗；未受击声道承担粗频分辨 + 更粗的 MS 时间粒度 + 额外两帧。

### 7.3 属于听感、本报告不下的结论

- MS 粒度变粗是否导致声像在瞬态附近"发飘"；
- 未受击声道的频率分辨率损失是否可闻（取决于该声道的频谱内容与掩蔽）；
- 分组在左右交替受击时的抖动是否可闻。

---

## 8. 听感判断（全部未验证）

> **本节内容没有经过主观试听，也没有仪器化感知模型验证。**
> 它是"若上述算法约束成立，则听感上可能如何"的**假设**，不能当作结论引用。

| 编号 | 听感假设 | 依据的算法约束 | 验证它需要什么 |
|---|---|---|---|
| L1 | 单侧瞬态若被合并进长窗，瞬态前会出现"沙沙"预回声 | 长窗把量化噪声摊到 23.2 ms | ABX：同一素材、强制 LONG vs 强制 START |
| L2 | 强制短窗后，非受击声道的稳态音色会略微"发糙" | 频率分辨率 43.1 → 344.5 Hz | 只测非受击声道的 MUSHRA |
| L3 | START/STOP 两帧的过渡本身可能产生轻微"软化" | 斜率从 N 切到 N/8 | 逐帧替换、听过渡帧 |
| L4 | 单侧受击时，声像在瞬态附近可能短暂塌向受击侧 | 公共分组 + 最大 `max_sfb` | 双声道 ABX + 声像定位测试 |
| L5 | 分组由"上一帧更响声道"决定，快速交替受击时可能听到周期性音色跳动 | `maxWindowNrg` 取自上一帧（`block_switch.cpp:275-280`） | 交替瞬态素材的长时听测 |

**【听感·未验证】** 上述五条都只是候选假设。特别是 L4：本报告**没有**任何证据表明 FDK 在单侧瞬态时会破坏声像；相反，5.2 节的契约保证了两声道的窗型一致，从算法上看声像并无结构性偏移来源。

**【听感·未验证】** 同样地，"短窗一定更好"不是结论。短窗用 8× 频率分辨率换 8× 时间分辨率；对持续音、稳态噪声、低频内容，长窗通常更优。具体素材上谁占优，只能用 L1/L2 的测试回答。

---

## 9. 约束清单（可直接用于评审）

**【约束·代码】** 改以下任何一条都会改变码流语义或破坏解码：

| # | 约束 | 位置 |
|---|---|---|
| C1 | `common_window` 恒为 1，左右共享单一 `ics_info` | `psy_main.cpp:413`, `:1163` |
| C2 | 左右各自独立检测，互不参考 | `psy_main.cpp:479-495` |
| C3 | 合并表必须是 `T[T[LONG][sL]][sR]`；只在 `sL, sR ∈ S(P)` 时保证合法 | `block_switch.cpp:411-437` |
| C4 | 合并结果对两声道同时满足 `fl == fr(prev)`——**前提是 C3 的输入不变量成立** | `block_switch.cpp:445-446` + `block.cpp:1035-1064` |
| C5 | 窗形在合并后再派生 | `block_switch.cpp:449-454` |
| C6 | 短窗分组必须同步，且 `noOfGroups ≤ 4` | `block_switch.cpp:461-532`, `psy_const.h:141` |
| C7 | `max_sfb` 共享，编码器须先取两声道最大 | `psy_main.cpp:1164-1166` |
| C8 | 短窗时 MS/IS 的索引跨度用同步后的分组 | `psy_main.cpp:1210-1214` |
| C9 | LC 下 `LOWOV` 不可达；混用返回 `-1` → `AAC_ENC_UNSUPPORTED_AOT` | `block_switch.cpp:563-570`, `psy_main.cpp:503-505` |
| C10 | LFE 强制 LONG + SINE，解码器拒绝非 LONG 的 LFE | `block_switch.cpp:237-247`, `channel.cpp:505-510` |
| C11 | `lastWindowSequence` 只能经 `chgWndSqLkAhd` 赋值（保持 C3 的输入不变量） | `block_switch.cpp:328-339` |

**【约束·代码】** 非契约、但会影响行为的实现细节：

- 分组平局时右声道胜出（严格 `>`，`block_switch.cpp:488`）；
- 右声道窗形使用**左声道**的 `allowShortFrames` 查表（`block_switch.cpp:452-453`）——两声道 AOT 相同，故无害，但是隐性耦合；
- `block_switch.cpp:523-531` 的 "start/stop" 分支不可达（4.4 节）；
- `attackIndex` 取最靠后的满足段，不是最强段（`block_switch.cpp:298-311`）；
- 合并表第一层 `T[LONG][·]` 是恒等行，当前冗余（4.2 节）；
- 单侧持续攻击会把短窗体制锁住，直到双方都停止攻击（5.5 节）。

---

## 10. 复现方法与证据

所有探针代码放在 `/tmp/wsharness`、测试音频与解析脚本放在 `/tmp/wstest`，**未修改仓库任何文件**。

### 10.1 构建

```sh
cmake -S . -B /tmp/fdkbuild -DCMAKE_BUILD_TYPE=Release -DBUILD_PROGRAMS=ON
cmake --build /tmp/fdkbuild -j10 --target aac-enc aac-dec
```

### 10.2 直接调用真实函数的探针

`/tmp/wsharness/harness.cpp` 等文件链接 `libfdk-aac`，直接调用 `FDKaacEnc_InitBlockSwitching / FDKaacEnc_BlockSwitching / FDKaacEnc_SyncBlockSwitching`。这样得到的是**实际代码行为**，不是阅读推断。

| 探针 | 用途 | 关键输出 |
|---|---|---|
| `probe3` | 帧级状态序列 | 单侧受击时另一侧被拖入 SHORT |
| `probe19` | 安静侧 `attack/lastattack` | 两值全程为 0，窗型仍被同步 |
| `probe5` | 5×5 合并矩阵穷举 | 得到第 4.1 节的表；`SHORT×LOWOV`、`LOWOV×SHORT` 返回 `-1` |
| `probe7` | 死分支验证 | "start/stop" 分支命中 0 次 |
| `probe8` | 代数性质（含 LOWOV） | 5×5 上 0 非交换、0 非幂等，2 个 `-1` 格 |
| `probe13` | 代数性质（LC 4×4） | 交换/幂等/结合均成立 |
| `probe10` | 分组归属 | 受击更响的一侧赢得分组 |
| `probe11` | 合法性闭包 | 合并输出 ⊆ `S(P)`，非法 0 |
| `probe12` | 平局规则 | 相等时右声道胜出 |
| `probe14` | 斜率集合 vs 状态机集合 | `S(P) ⊆ L(P)`；`P=START` 为真子集 |
| `probe15` | `SHORT×STOP` 恒为 SHORT | 单侧持续攻击锁住短窗 |
| `probe16` | 最短短窗往返 | 4 帧：`START→SHORT→STOP→LONG` |
| `probe17` | 4×4 子表对称性 | 非对称格 0 个 |
| `probe18` | 合并表的域外负面对照 | 16 格中 12 格斜率不合法（契约只在 `S(P)` 内成立） |

示例（`probe10`，左声道 f4 第 1 段受击、右声道 f4 末段受击）：

```
f5 pre L=SHORT grp=4(1,1,3,3) maxNrg=707328381 lastIdx=1 | R=SHORT grp=4(3,3,1,1) maxNrg=739218606 lastIdx=6
   -> post SHORT grp=4(3,3,1,1)
```

说明：两声道都被合并为 SHORT（正确），分组取 `maxWindowNrg` 更大的右声道；而右声道触发合并的其实是 f4 末段的攻击。

### 10.3 端到端码流验证

合成两段 1 秒立体声（左声道 0.5 s 处有衰减瞬态 / 两声道都有），编码为 AAC-LC 96 kbit/s，再用脚本按 ADTS 语法解析每个 CPE 的 `common_window / window_sequence / window_shape / scale_factor_grouping`：

```sh
aac-enc -t 2 -r 96000 left_transient.wav left.aac
aac-enc -t 2 -r 96000 both_transient.wav both.aac
python3 -I parse2.py left.aac both.aac
```

结果：两文件的窗序列在受击区完全一致（共 5 帧：START / EIGHT_SHORT / EIGHT_SHORT / STOP / LONG），验证了 C1 与 C4 在真实码流上的成立；`window_shape` 依次为 `0,0,0,1,1`（SINE,SINE,SINE,KBD,KBD），与 `blockType2windowShape` 的 LC 行一致。

【推断·可测】帧数也与"最短 4 帧、通常 5 帧"的预期吻合（3 节）：这里的 5 帧对应 `START + 2×SHORT + STOP + LONG`，是跨帧延续规则（2.3 节）把攻击延长了一帧的结果。

---

## 11. 未覆盖 / 不适用

- **AAC-LD / ELD**：走 `chgWndSq`（无 look-ahead），可能出现 `_LOWOV_WINDOW`，并在 `block_switch.cpp:563-579` 被翻译成 `LONG + LOL_WINDOW`。其"可解码关系"由 `LOL` 的 3/4 零重叠单独定义（`block.cpp:969-978`），与本文的 LC 契约不同，本文未展开。
- **USAC / USAC 的 `common_max_sfb`**：`channel.cpp:519-523` 的分支仅对 USAC 生效；LC 无此语法元素，ch1 的 `max_sfb` 纯继承。
- **MPEG-2 AAC**：LFE 与窗型约束相同，但本文未单独验证。
- **主观听感**：完全未做，见第 8 节。
