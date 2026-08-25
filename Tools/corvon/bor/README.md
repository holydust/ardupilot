# CORVON BOR option byte 工装脚本

**可执行件的真相源在本目录。** `corvon-hq/work/g-series-gps/DW608_BOR_OptionByte_配方.md`
是**说明书**(为什么必须 Level 3、三源寄存器对照、红线依据、开口项),讲道理不放代码;
两处若不一致,**以本目录为准**,并回头修说明书。

放在固件仓的理由:脚本跟着 commit 走,`git_identity` 天然覆盖 ——「固件 + 脚本」才是一个
可版本化的单元,产线不可能拿到错配的组合。

## 这是干什么的

出厂默认 BOR 是**关**的。必须设成 **Level 3(约 2.7V)**,否则 VDD 掉到 2.7V 以下时 MCU 仍在跑,
而 ArduPilot 的 F4 flash 擦除**永远用 x32 并行度**(`flash.c`,无 x16 退路),RM0090 Table 13 规定
x32 只允许 VDD 2.7–3.6V,超出时数据 *"may not be retained"* —— 会**静默写坏**参数或固件。

flash 写不只发生在产线:串口 `set`+`save`、飞控经 DroneCAN 改参、DroneCAN 固件升级,每次都含 x32 擦除。

## 产线用法:不要单独跑这些脚本

**烧录脚本已经把 BOR 并进去了**,产线只记一条命令:

```
Tools/corvon/flash/flash-swd.sh g1        # 探测 -> 烧写 -> verify -> 回读比对 -> 设 BOR -> 查 BOR
```

裸板第一次本来就要用 SWD 烧固件,**探针已经在焊盘上**,BOR 的增量成本约 2 秒,不需要独立工位。
放行判据是脚本最后那句 `BOR CHECK PASS`。`--no-bor` 只给工程调试用,**跳过的板子不可出货**。

放行检查用 `bor_check.cfg`(见下),**不需要断电、不需要万用表**。下面那套两级 gate 保留给工程验证。

## 为什么产线不用两级 gate 了

Gate 2 要求受控断电 + 实测放电电压,产线上太重,而且它唯一的失效模式恰恰是"放电不透 → 假通过"——
一个既昂贵又容易被糊弄过去的判据。

`bor_check.cfg` 改为**直接读 option byte 的 flash 存储区**,拿到的是存储单元内容而不是加载副本,
比"断电后重读加载副本"更直接地回答了同一个问题,且没有可被糊弄的余地。

```
openocd -f interface/stlink-dap.cfg -f corvon-f405-swd.cfg \
        -c "set SILICON geehy" -f bor_common.cfg -f bor_check.cfg
```

它做三件事:补码自洽校验(证明写入完整未截断)、RDP 必须 `0xAA`、`BOR_LEV` 必须 `0b00`,
最后与加载副本交叉核对。任一不符 `exit != 0`。

🟡 **标定数据已取到,退役决定尚未做出 —— `bor_verify.cfg` 现在仍要跑。**

2026-08-25 在 G1(UID `000D001D 3531340E 43485053`)上,离台数日、当天又断过一次电之后,
不发复位直接读:

```
0x1FFFC000 = 0x550CAAF3    value 0xAAF3  RDP 0xAA  BOR_LEV 0(=Level 3)  补码 0x550C = ~0xAAF3 ✓
OPTCR      = 0x0FFFAAE1    (加载副本,一致)
```

连同已有的两条,证据链是:

| | 观察 | 排除了什么 |
|---|---|---|
| ① | G2 2026-08-20:只写 `OPTCR`、不发 `OPTSTRT`,`0x1FFFC000` 不动 | 它不是 `OPTCR` 的别名 |
| ①' | 每块过 `flash-swd.sh` 的板子:`OPTSTRT` 后未复位,`0x1FFFC000` 已是新值 | 它不是复位时锁存的快照 |
| ③ | 上面这次:真断电后仍读回写入值,补码自洽 | 值在非易失存储里 |

三条合起来,"读的是 option byte 存储单元"是唯一还站得住的解释。
**但请注意力度分布:决定性的是 ①**(制造存储与寄存器不一致,再看它跟谁);
③ 补的是非易失这条腿,单独看不足以定案。

弱旁证(**不要单独引用**):同次读到 `0x1FFFC008` = `0x0000FFFF` 而 `OPTCR1` = `0x00000000`,
两者不一致 —— 但也可能只是两个未实现地址的总线默认值不同。

🔴 **未决**:撤掉一道产线安全门属于不好回滚的决定,需要一次对抗性复核 + Vince 拍板。
在那之前两级 gate 继续并行跑。

🔴 **已知缺陷**:`bor_check.cfg` 在 `halt` 之后不 `resume`,会把单元撂在停机态。
产线上单元随后本来就要断电,影响待定;但工程台上这会让人误判为"板子挂了"。
纯读取走 AHB-AP,核在跑时就能读,其实不需要 `halt` —— 见
`Tools/corvon/diag/README.md` 2026-08-25 那条。

## 工程验证用法(两级 gate)

**探针 cfg 必须是 dap 型**:本目录的 `corvon-f405-swd.cfg` 用 `dap create`,配 ST-Link 时是
`interface/stlink-dap.cfg`,**不是 `interface/stlink.cfg`**(后者是 hla 驱动,自己占着 DAP)。

```
# Gate 1:写入 + 即时校验
openocd -f interface/stlink-dap.cfg -f corvon-f405-swd.cfg \
        -c "set SILICON st" -f bor_common.cfg -f bor_program.cfg

# —— 受控断电:VDD 实测放电 <0.3V 并驻留 ≥500ms ——

# Gate 2:重上电后复读(**这一步才是放行判据**)
openocd -f interface/stlink-dap.cfg -f corvon-f405-swd.cfg \
        -c "set SILICON st" -f bor_common.cfg -f bor_verify.cfg
```

`SILICON` 由工单注入,取 `st` / `geehy` / `gd`。**认证硅源 = Geehy APM32F405RGT6(主力)+ ST(备)**;
`gd` 分支为应急未认证库存保留,**产线常规禁用**。

任一 gate `exit != 0` = FAIL/隔离。

## 🔴 放电时必须断开所有回灌路径(2026-08-19 实测补充)

只拔主供电不够。**ST-Link 的 SWCLK/SWDIO 和 USB-TTL 的 TX 都是 3.3V 推挽输出**,
会经 MCU 引脚的 ESD 上二极管往 VDD 灌电,把电压顶在复位阈值以上,**放电永远到不了 0.3V**。

放电时必须**同时断开**:4P 供电、SWD 探针、串口转接板。建议一根一根拔、边拔边看表,
顺带测出每条回灌路径各自撑住多少电压 —— 治具定型时仍建议记录这组数。

## 另一条更强的判据:直接读 flash 里的 option byte

Gate 2 的唯一失效模式是"放电不透 → 读到影子值 → 假通过"。有个不依赖断电的独立验证:
**option byte 在 flash 里的固定地址是 `0x1FFFC000`**,那里是真正的存储单元,不是寄存器影子。

STM32 以「值 + 按位取反」成对存储:

```
0x1FFFC000 = 0x550CAAF3
  低半字 0xAAF3 = 实际值   RDP=0xAA(bits 15:8)  用户字节 0xF3 -> bits[3:2]=0b00 = Level 3
  高半字 0x550C = 补码     ~0xAAF3 = 0x550C 自洽 -> 写入完整未被截断
```

**互补校验自洽 + BOR_LEV 正确 = flash 单元确实持有 Level 3。** 建议工装把这一条也纳入,
与 Gate 2 互为独立佐证(2026-08-19 首片 DW608 即用此法确认)。

## 为什么要两级 gate

**Gate 1 读回来的是寄存器影子值,不是 flash 里的内容。** 只有真断电、让 option byte 从 flash 重新加载,
读到的才作数。而且断电必须受控 —— 板上 4.7µF 会把 VDD 撑在复位阈值以上,电压没掉透 option byte 就不重载,
Gate 2 读到的还是影子值,**等于白测**。

## 安全红线(逐条进工装评审)

1. **RDP/SPC [15:8] 任何时刻必须 `0xAA`**;`0xCC` = 永久锁死。操作前红线 + 两级回读三重保险
2. **写入禁止整字常数,必须 RMW**;验收才用整字策略比对
3. **先 halt**;操作期间固件不得收到写参数指令
4. **BUSY 期间严禁断电 / 断继电器 / 抬探针**。超时输出 `QUARANTINE_HOLD_POWER`,工装必须锁存:
   保持 VDD 与探针、禁止继电器动作、禁止自动重试,持续监测 BUSY,人工处置。
   **上层看到 `exit != 0` 一律不得自动断电重试**
5. **本次 OB 写自身没有 BOR 保护**:序列启动前实测 VDD 合格,写入窗口内供电 hold-up
6. **Gate 2 的断电必须受控**(见上)
7. 幂等路径同样跑全量校验与 Gate 2;失败路径 best-effort 重锁
8. **工装配置整体冻结**:探头型号、adapter driver、speed、transport、reset 配置、OpenOCD 版本、
   命令行、脚本 hash,一经 spike 验证禁止产线替换。另加**进程级 watchdog**
   (Tcl 超时管不住 SWD 底层调用卡死;杀 OpenOCD 进程不等于断板电,电源仍由 QUARANTINE 状态机管)

## 供电

**建议全程从 4P CAN 口供 5V**,ST-Link 只接 `G`/`C`/`D` 不接 `V`。
`V` = 板载 `3V3` 主轨(U5 HL6333M5R 输出),外部再灌 3.3V 会与 LDO 对顶。
详见 `DW608_产测方案.md` §2-4。

## 已闭环

### 一、`0x1FFFC000` 的读取不跟随 OPTCR —— 2026-08-20,G2 首片

配方担心的失效模式是:Gate 1 回读到的只是**寄存器影子**(`OPTCR`),flash 没写进去
也照样读出正确值(见配方 §Gate 2「Gate 1 的回读只是寄存器影子值」)。直接验这条:
解锁 OPTCR 后**只改寄存器里的 BOR_LEV、不置 OPTSTRT**(因此零 flash 写入),重读:

```
OPTCR       0x0FFFAAED → 0x0FFFAAE0    寄存器已改成 Level 3
0x1FFFC000  0x5500AAFF → 0x5500AAFF    该地址纹丝不动
```

随后真正写入 option byte,同一地址变成 `0x550CAAF3`。**两个方向都成立 →
`0x1FFFC000` 不是 OPTCR 的镜像**,而这正是 Gate 2 当初要绕开的那条路径。

> **措辞边界(2026-08-24 收窄)**:本实验证明的是"不是 **OPTCR** 的镜像"。若假设
> 存在一个**独立于 OPTCR、由编程硬件更新的第三方影子**,本实验管不到它 —— 那种情况
> 只有断电重载能排除。按配方自身的风险模型(影子 = 寄存器)本条成立;按更严的读法
> 仍留一个缺口。初版写成"读的是 flash 存储不是加载副本",越过了证据,已改。

### 二、写入确实落到了 flash —— 2026-08-21,G1 实测

G1 于 08-19 写入 BOR,随后**断电、离台、交测试组数日**;08-21 重新上电后读到:

```
FLASH_OPTCR = 0x0FFFAAE1     BOR_LEV = 0b00 = Level 3(出厂默认 0x0FFFAAED)
```

`OPTCR` 是复位时从 option byte flash 加载的。**一次货真价实的长时间断电之后它仍是
Level 3 → flash 单元确实写进去了。** 这比脚本设想的受控放电更硬:放电深度不必测量,
板子断了好几天,也不存在探针/串口的回灌路径。

### 三、结论与仍欠的一步

`bor_check.cfg` 作为放行判据成立。**但 `bor_verify.cfg` 暂不退役**,直到补上最后
一次测量:**断电重上之后读一次 `0x1FFFC000`**(注意是这个地址,不是 `OPTCR`)。
只有它能把上面那个"第三方影子"的缺口彻底堵死。**这一步 30 秒,下次有板子在台上
顺手做掉即可。**

边界:单片样本、单一硅源(APM32 `0x0009A413`)。

## 尚未闭环

- **断电重上后读一次 `0x1FFFC000`** —— 见上「三」,`bor_verify.cfg` 在此之前保留
- 三源各 2–3 片实测(DBG_ID 回填、option 擦写耗时)
- 首片实测参考值:`DBG_ID 0x0009A413`、`FLASH 1024 KB`、写前 `OPTCR 0x0FFFAAED`、
  写后 `0x0FFFAAE1`、`FLASH_SR` 清洁、`Target voltage 3.212V`
- SWD 焊盘是否引出 NRST —— 有则 `reset_config` 改 `srst_only`
- EVT 压降瞬态:3.3V 轨跌落不得误触 2.88V(最坏)
