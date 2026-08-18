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

```
# Gate 1:写入 + 即时校验
openocd -f interface/<probe>.cfg -f corvon-f405-swd.cfg \
        -c "set SILICON st" -f bor_common.cfg -f bor_program.cfg

# —— 受控断电:VDD 实测放电 <0.3V 并驻留 ≥500ms ——

# Gate 2:重上电后复读(**这一步才是放行判据**)
openocd -f interface/<probe>.cfg -f corvon-f405-swd.cfg \
        -c "set SILICON st" -f bor_common.cfg -f bor_verify.cfg
```

`SILICON` 由工单注入,取 `st` / `geehy` / `gd`。**认证硅源 = Geehy APM32F405RGT6(主力)+ ST(备)**;
`gd` 分支为应急未认证库存保留,**产线常规禁用**。

任一 gate `exit != 0` = FAIL/隔离。

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

- 三源各 2–3 片实测(DBG_ID 回填、Gate 2 放电阈值/驻留标定、option 擦写耗时)
- SWD 焊盘是否引出 NRST —— 有则 `reset_config` 改 `srst_only`
- EVT 压降瞬态:3.3V 轨跌落不得误触 2.88V(最坏)
