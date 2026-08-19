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

## 用法

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
顺带测出每条回灌路径各自撑住多少电压 —— 这正是下面"尚未闭环"里要标定的数。

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

## 尚未闭环

- 三源各 2–3 片实测(DBG_ID 回填、option 擦写耗时)
- **Gate 2 放电阈值/驻留标定仍未做**:2026-08-19 首片走完全流程,但断电未实测电压
  (改用上面的 flash 直读作为替代证据)。治具定型前必须补一次带测量的断电,记录:
  拔 4P 后稳在多少(回灌量)、三样全拔后最低多少、掉破 0.3V 需要多久
- 首片实测参考值:`DBG_ID 0x0009A413`、`FLASH 1024 KB`、写前 `OPTCR 0x0FFFAAED`、
  写后 `0x0FFFAAE1`、`FLASH_SR` 清洁、`Target voltage 3.212V`
- SWD 焊盘是否引出 NRST —— 有则 `reset_config` 改 `srst_only`
- EVT 压降瞬态:3.3V 轨跌落不得误触 2.88V(最坏)
