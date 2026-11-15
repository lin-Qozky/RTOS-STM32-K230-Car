

# RTOS-STM32-K230-Car

### 功能:五路循迹、避障停止、k230神经网络训练识别物块、机械臂抓取物块、二维码识别停止。
### 视频链接:【基于RTOS的智能小车】 https://www.bilibili.com/video/BV1HzsE69EsD/?share_source=copy_web&vd_source=bca16ed409e00bf0aaf06bd9f30b66f4
### 目录
- [0. 这份文档怎么读](#0-这份文档怎么读)
- [1. 代码逻辑总览](#1-代码逻辑总览)
- [2. 函数之间怎么配合](#2-函数之间怎么配合)
- [3. 语法讲解](#3-语法讲解)
- [4. 注释与实际代码不一致的地方](#4-注释与实际代码不一致的地方)

---

### 0. 这份文档怎么读

建议按 **任务表 → 状态量 → 时序** 的顺序看：先知道"有哪几个人在干活"，再看"他们通过哪几个变量传话"，最后看"一次抓取从头到尾发生了什么"。语法部分可以单独当字典查。

---

### 1. 代码逻辑总览

#### 1.1 硬件资源分配

文件开头的注释已经把引脚列全了，对应到驱动如下：

| 外设 | 引脚 | 驱动文件 | 说明 |
|---|---|---|---|
| 电机 A（右轮） | PA5/PA4 + PWM PB6 | Motor.c | `Motor_SetSpeed()` |
| 电机 B（左轮） | PB4/PB5 + PWM PB7 | Motor.c | `Motor_SetSpeed2()` |
| OLED | PB12/PB13 | OLED.c | 软件 I2C，显示距离 |
| 舵机 1~4 | PA0~PA3 | PWM_Servo.c | TIM2 CH1~CH4，别名 `Servo1` |
| 舵机 5、6 | PB0/PB1 | PWM_Servo.c | TIM3 CH3/CH4，别名 `Servo2`，只用 CH4（舵机6） |
| 串口 | PA9/PA10 | Serial.c | USART1，115200，中断收字节 |
| 超声波 | PA6(TRIG)/PA7(ECHO) | ultrasonic.c | 单次最长阻塞约 50ms |
| 五路循迹 | PB8/PB9/PB10/PB11/PB14 | xunji.c | `Xunji_Control()` |

关键点：**没有第二个地方直接写电机**，除了 `stop` 指令之外，所有电机输出都集中在 `Xunji_Task` 这一个任务里跑，这是这份代码最重要的设计约束（见 2.3）。

#### 1.2 软件分层

```
应用层   main.c（任务编排 + 抓取状态机 + 坐标跟踪）
          │
驱动层   Motor / PWM_Servo / xunji / serial / ultrasonic / OLED
          │
系统层   FreeRTOS（5 个任务 + 1 个空壳 Task6）
          │
硬件层   STM32F103 + 标准库
```

`main.c` 只做三件事：
1. **初始化**（1.3 节之后讲）；
2. **建任务**，每个任务是一个死循环 + 固定周期；
3. **用全局状态量做任务间"传话"**，不直接跨任务调函数。

#### 1.3 五个任务

| 任务函数 | 任务名 | 周期 | 优先级 | 栈 | 干什么 |
|---|---|---|---|---|---|
| `Ultrasonic_Task` | Ultrasonic | 100ms | 2 | 256 | 读超声波，写 `g_DistanceCm`，OLED 显示 |
| `Avoid_Task` | Avoid | 100ms | 2 | 256 | 读 `g_DistanceCm`，太近就停车并关循迹 |
| `Xunji_Task` | Xunji | 10ms | 3 | 256 | **唯一的电机执行层**：循迹 / 坐标跟踪 / 停车 |
| `CatchLitter_Task` | Catch | 12ms | 3 | 256 | 每周期调一次 `catch_litter()`，推进舵机状态机 |
| `Serial_Task` | Serial | 50ms | 1 | 256 | 回显原始字节、解析颜色坐标指令、改状态量 |

优先级含义：数字越大越优先。`Xunji_Task` 和 `CatchLitter_Task` 是 3，`Ultrasonic/Avoid` 是 2，`Serial` 是 1。也就是**电机和舵机的动作永远不会被串口解析打断**，而串口可以随时被抢。

> FreeRTOS 抢占式调度：优先级高的任务一旦就绪立刻运行，所以 10ms 的循迹任务实际抖动很小，串口任务被推迟几十毫秒也无所谓。

#### 1.4 三条主线

整份 main.c 就是三套动作在抢同一个车：

```
主线A 循迹：      g_XunjiEnable=1 ──► Xunji_Control() 每 10ms
主线B 跟踪抓取：  g_TrackEnable=1 ──► CameraTrack_Control() ──► g_ArmDelayTicks ──► catch_litter()
主线C 避障/急停： g_AvoidEnable=1 / g_StopFlag=1 ──► 双电机 0
```

三个开关的互斥关系：

| 场景 | g_XunjiEnable | g_AvoidEnable | g_TrackEnable | g_CatchRunning |
|---|---|---|---|---|
| 上电默认 | 1 | 1 | 0 | 0 |
| 收到颜色坐标 | 0 | 0 | 1 | 0 |
| 到窗口中心停稳 | 0 | 0 | 0 | 0 →（1s后）1 |
| 抓取过程中 | 0 | 0 | 0 | 1 |
| 抓取收尾完成 | 1 | 1 | 0 | 0 |
| 收到 `@stop` | 0 | 0 | 0 | 不变 |

#### 1.5 一次完整抓取的事件时序

```
K230 发 "@red,291,230,1\r\n"
        │ USART1 中断逐字节拼包
        ▼
Serial_RxFlag = 1，Serial_RxPacket = "red,291,230,1"
        │ Serial_Task（≤50ms 内）
        ▼
解析 4 个字段 → 颜色映射 stage4Angle=110、gripAngle=20
写 g_ObjX/g_ObjY/g_ObjColor，g_ObjFreshTicks=50，g_TrackEnable=1
关循迹、关避障、双电机 0、Xunji_Reset()
        │
        ▼
Xunji_Task 每 10ms 调 CameraTrack_Control()
  |x-413| ≤ 40 且坐标新鲜 → 停车，g_ArmDelayTicks=84（≈1.0s）
  |x-413| > 40          → 双轮同速前进或后退（不转向）
  坐标过期(50×10ms)      → 放弃跟踪，恢复循迹
        │ 84×12ms ≈ 1.0s
        ▼
g_CatchRunning=1，s_CatchStage=0
        │ CatchLitter_Task 每 12ms 调 catch_litter()
        ▼
阶段 1~12 串行：每个舵机每步 ±1 度，阶段间停 42×12ms ≈ 500ms
  · 阶段6 舵机1 目标 = g_Servo1DownTarget（红110/粉90/蓝75）
  · 阶段7 舵机4 目标 = g_ClawGripTarget（固定20，夹紧）
        │
        ▼
全部阶段完成 → g_CatchRunning=0
  · s_FinishTicks = 42（收尾停留 ≈500ms）
  · g_SerialStopTicks = 20（收尾后 20×50ms ≈1s 丢弃新指令）
        │
        ▼
收尾倒计时到 0 → g_XunjiEnable=1、g_AvoidEnable=1、Xunji_Reset()，回到主线A
```

两个"防抖"设计值得注意：
- **`g_ObjFreshTicks`**：K230 每帧都会发坐标，超过 500ms 没收到就认为目标丢了，自动退回循迹。它只在 `CameraTrack_Control()` 里递减，也就是只有在跟踪状态下才会倒计时。
- **`g_SerialStopTicks`**：抓取刚结束时，K230 发来的还是同一个物体的旧坐标，如果不屏蔽会立刻再抓一次。所以收尾后再"堵住"约 1 秒。

#### 1.6 main() 里的初始化顺序

```c
MOTOR_Init();          // 电机 PWM + 方向脚
PWM_Servo_Init();      // TIM2/TIM3 六路 PWM
Infrared_Init();       // 五路循迹 GPIO
Ultrasonic_Init();     // PA6/PA7
Serial_Init();         // USART1 + RXNE 中断

Angle_SetCompare(Servo1, 1, 180); Delay_ms(500);   // 舵机1 抬到 180
Angle_SetCompare(Servo1, 2, 100); Delay_ms(500);   // 舵机2 100
Angle_SetCompare(Servo1, 3, 180); Delay_ms(500);   // 舵机3 180
Angle_SetCompare(Servo1, 4, 0);   Delay_ms(500);   // 舵机4 夹爪张开
Angle_SetCompare(Servo2, 4, 115); Delay_ms(500);   // 舵机6 115

OLED_Init();  OLED_ShowString(1,1,"Dist:");  ...
Xunji_Reset();                                     // 清 PID 状态
xTaskCreate(...) × 5;
vTaskStartScheduler();                             // 之后再也不会返回
while (1) { }
```

这 5 个 `Angle_SetCompare` 的顺序和角度**必须和 3.7 节讲的"软件影子角度"一致**，否则第一次抓取会先抽搐一下再动。`Delay_ms` 用的是裸机延时，此时调度器还没启动，可以直接用。

---

### 2. 函数之间怎么配合

#### 2.1 共享状态一览（谁写、谁读）

| 变量 | 写方 | 读方 | 作用 |
|---|---|---|---|
| `g_DistanceCm` | Ultrasonic_Task | Avoid_Task | 最新距离，-1 = 无回波 |
| `g_XunjiEnable` | Serial/Avoid/catch_litter | Xunji_Task | 循迹总开关 |
| `g_AvoidEnable` | Serial/catch_litter | Avoid_Task | 避障总开关 |
| `g_TrackEnable` | Serial/CameraTrack | Xunji_Task | 坐标跟踪开关 |
| `g_CatchRunning` | catch_litter | Serial_Task、catch_litter | 抓取进行中 |
| `g_StopFlag` | Serial_Task（只置 1） | 所有任务 | 全局急停，只增不减 |
| `g_ObjX/g_ObjY/g_ObjColor/g_ObjCount` | Serial_Task | 日志/预留 | 目标信息存档 |
| `g_ObjFreshTicks` | Serial_Task 置 50 | CameraTrack 递减 | 坐标保鲜期 |
| `g_Servo1DownTarget` | Serial_Task | catch_litter 阶段6 | 按颜色决定下压角 |
| `g_ClawGripTarget` | Serial_Task | catch_litter 阶段7 | 按颜色决定夹紧角 |
| `g_SerialStopTicks` | catch_litter 置值 | Serial_Task 递减 | 抓取后停收指令 |
| `s_CatchStage/s_SettleTicks/s_FinishTicks/g_ArmDelayTicks` | catch_litter / CameraTrack / Serial | 同左 | 舵机状态机 |
| `s_Servo1Angle[]/s_Servo2Angle[]` | catch_litter 自增自减 | catch_litter | 舵机当前角度影子 |

规律：**所有跨任务的变量都带 `volatile`，且都是 8/16 位整数或 float 的"单次读写"**。没有用队列、信号量，因为每个变量只有一个写方（除 `g_XunjiEnable` 有三个写方，但每个写方只写 0/1，天然原子）。

#### 2.2 Ultrasonic_Task ↔ Avoid_Task：生产者/消费者

```
Ultrasonic_Task(100ms)                 Avoid_Task(100ms)
   distance = Ultrasonic_GetDistance()
   g_DistanceCm = distance  ─────────►  distance = g_DistanceCm
   OLED 显示                                    │
                                    distance ≤ 20cm?
                                      ├ 是 → g_XunjiEnable=0; 双电机 0
                                      └ 否且刚才被关过 → g_XunjiEnable=1; Xunji_Reset()
```

- 两个任务周期都是 100ms，**没有做同步**，所以 `Avoid_Task` 可能读到上一周期的距离（最多晚 100ms），对 20cm 的阈值来说可以接受。
- 避障只负责"关/开循迹开关"，**不碰电机速度以外的逻辑**；恢复时调 `Xunji_Reset()` 清掉 PID 的积分和上次误差，防止停车期间积分饱和导致一恢复就猛打方向。
- `distance < 0`（无回波）时 `continue`，避免把"没测到"误判成"很近"。

#### 2.3 Xunji_Task ↔ CameraTrack_Control：单执行点 + 状态分发

`Xunji_Task` 是**唯一会周期性写电机的地方**，它内部按优先级选一个分支：

```c
if (g_StopFlag)            { 停车; }              // 1. 急停最高
else if (g_TrackEnable)    { CameraTrack_Control(); } // 2. 坐标跟踪
else if (g_XunjiEnable)    { Xunji_Control(); }        // 3. 循迹
else                       { 停车; }              // 4. 都不开 → 停
```

这个"单执行点"结构带来两个好处：
1. 不会出现"循迹任务让它前进、串口任务让它后退"的打架；
2. 模式切换只需要改开关量，不用考虑时序。

`CameraTrack_Control()` 的三条出口：

| 条件 | 动作 | 退出到 |
|---|---|---|
| `g_ObjFreshTicks > 0` 且 `dx ∈ [-40, +40]` | 双电机 0，`s_CatchStage=0`，`g_ArmDelayTicks=84` | 等抓取 |
| `dx ≤ -40`（目标偏左） | `Motor_SetSpeed(1500)` + `Motor_SetSpeed2(1500)` 前进 | 继续跟踪 |
| `dx > +40`（目标偏右） | 双轮 `-1500` 后退 | 继续跟踪 |
| `g_ObjFreshTicks == 0` | 停车 + `g_XunjiEnable=1`、`g_AvoidEnable=1`、`Xunji_Reset()` | 回循迹 |

**为什么是前进/后退而不是转向**：盯的是像素横坐标，而像素横坐标随"车与目标的距离"变化（靠近时目标在画面里横向位移），所以用直线逼近把 x 拉回 413，而不是差速转弯。左右轮给同一个速度值，`Motor_SetSpeed2` 是左轮、`Motor_SetSpeed` 是右轮，两者同号即直行。

#### 2.4 Serial_Task：只改状态，不当司机

`Serial_Task` 的处理顺序是严格分层的，这个顺序本身就是逻辑：

```c
1. 取出原始字节并原样回发        ← 调试用，K230 或串口助手能看到自己发了什么
2. 收到 "stop"                    ← 立刻停车，continue，后面的解析全跳过
3. g_SerialStopTicks > 0          ← 抓取刚结束，清 RxFlag 丢弃指令，continue
4. sscanf 解析 "颜色,x,y,count"   ← 4 个字段都要成功
5. 颜色映射                       ← 不认识的颜色的 stage4Angle 保持 255
6. 三道过滤                       ← 正在抓取/已排队、y 越界 → 什么也不做
7. 写状态量、开跟踪、关循迹避障
```

为什么第 2 步要放在第 3 步前面：`@stop` 是"救命"指令，抓取后的 1 秒屏蔽期也必须能停车，所以急停优先于屏蔽。

第 6 步的两道过滤：
- `g_CatchRunning || g_ArmDelayTicks > 0`：已经在抓或已经排队，忽略新目标（否则中途改目标会让舵机来回跳）。
- `y < 0 || y > 300`：目标纵向位置不合理（太近/太远），忽略，等下一帧。

第 7 步只做三件事：存目标、置保鲜期、切开关。**不动电机**，电机由 10ms 的 `Xunji_Task` 下一拍执行，这样即使串口任务被抢占也不会出现速度写一半的情况。

#### 2.5 catch_litter：四层状态机

`catch_litter()` 被 `CatchLitter_Task` 每 12ms 调一次，函数内部有 4 个"闸门"，从上到下依次判断，命中就 `return`：

```
① s_FinishTicks > 0        收尾阶段：倒计时，归 0 时恢复循迹避障，return
② !g_CatchRunning          待机阶段：
     g_ArmDelayTicks > 0 则递减，减到 0 才 g_CatchRunning=1、s_CatchStage=0
     return（注意：这里无论如何都 return）
③ s_SettleTicks > 0        阶段间停留：递减后 return
④ 主推进                   当前阶段所有舵机各走 1 度；全部到位则 → 下一阶段 或 结束
```

第 ④ 步的细节：

```c
stage  = &s_Stages[s_CatchStage];      // 取当前阶段的动作表
for (每个要动的舵机) {
    angle  = Servo_AnglePtr(TIMx, ch); // 拿到影子角度地址
    target = Move[i].target;           // 表里的默认目标

    if (s_CatchStage == 5) target = g_Servo1DownTarget;  // 阶段6：颜色覆盖
    if (s_CatchStage == 6) target = g_ClawGripTarget;    // 阶段7：颜色覆盖

    *angle 向 target 靠 1 度;          // 就地修改影子角度
    Angle_SetCompare(TIMx, ch, *angle);
    if (*angle != target) allArrived = 0;
}
if (allArrived) {
    最后一阶段？ → 结束：g_CatchRunning=0, s_FinishTicks=42, g_SerialStopTicks=20
    否则         → s_CatchStage++, s_SettleTicks=42
}
```

几个要点：

1. **一阶段只动 1 度**是刻意的：舵机响应慢，一次性给终点会甩头，12ms 一步相当于限速。
2. **阶段之间停留** `CATCH_SETTLE_TICKS = (500+12-1)/12 = 42` 拍，约 500ms，让机械臂稳定下来再走下一步。
3. **颜色只在阶段 6、7 生效**：`s_Stage6[]`/`s_Stage7[]` 里写的 90 只是占位，运行期被 `g_Servo1DownTarget` / `g_ClawGripTarget` 覆盖。这样颜色映射全集中在 `Serial_Task` 一处，动作表不用改。
4. **结束时的两个计时器分工**：`s_FinishTicks`（12ms 基准）让动作本身有个收尾停留；`g_SerialStopTicks`（50ms 基准，20 拍≈1s）覆盖"收尾 + 恢复循迹后的 500ms"，防止 K230 旧目标立刻再触发。
5. **`s_Servo1Angle[]` / `s_Servo2Angle[]` 是软件影子**：因为标准库的 PWM 比较寄存器读回来不方便，代码自己记当前位置。数组下标就是通道号（下标 0 不用），`Servo_AnglePtr()` 负责二选一（TIM2 还是 TIM3）。
6. **上电角度必须与影子初值一致**：`{0,180,100,180,0}` 与 `{0,0,0,0,115}` 对应 main 里的 5 个初始化，不一致会导致第一步"跳档"。

#### 2.6 三条主线的冲突处理总结

| 冲突 | 处理方式 |
|---|---|
| 循迹 vs 跟踪 | 同一个任务里 `if/else if`，天然互斥 |
| 跟踪 vs 抓取 | 到中心后 `g_TrackEnable=0`，1 秒后才 `g_CatchRunning=1` |
| 抓取 vs 循迹 | 抓取期间 `g_XunjiEnable=0`，结束后由 `s_FinishTicks` 统一恢复 |
| 避障 vs 跟踪 | `Serial_Task` 收到目标时把 `g_AvoidEnable=0`，抓完再打开 |
| 急停 vs 一切 | `g_StopFlag` 在四个任务里都是第一判断，且没有任何地方清 0 |
| 旧指令 vs 新一抓 | `g_SerialStopTicks` 屏蔽窗口 |

#### 2.7 Task6 空壳的位置

`Task6_Task` 现在整段被 `/* ... */` 包住，位于 `CatchLitter_Task` 之后、`main()` 之前；`Task6Handler` 和 `xTaskCreate` 也分别注释着。启用时四处一起放开：

1. 周期宏 `TASK6_PERIOD_MS`
2. 句柄 `TaskHandle_t Task6Handler;`
3. 任务函数体（在 `while(1)` 里填你自己的函数调用）
4. `main()` 里的 `xTaskCreate(Task6_Task, ...)`

注意壳子里已经带了 `if (g_StopFlag) continue;`，和现有五个任务保持同一套急停约定。

---

### 3. 语法讲解

#### 3.1 `static` 函数

```c
static void Xunji_Task(void *arg)
```

`static` 加在函数前，表示**内部链接**：这个符号只在 `main.c` 里可见，其他 .c 文件链接不到。好处是重名安全（别的文件也可以有个 `Serial_Task`），还能让编译器放心内联/裁剪。文件末尾的 `catch_litter()` 没有 `static`，因为它在文件开头声明过、并且是唯一对外的一个函数（虽然目前也没人外部调用）。

#### 3.2 `volatile`

```c
static volatile float g_DistanceCm = -1.0f;
```

告诉编译器：这个变量可能在当前代码流之外被改（中断、另一个任务），**每次用都必须重新从内存读，不许缓存到寄存器、不许做删除优化**。没有它，`while (g_SerialStopTicks > 0);` 这种循环可能被优化成死循环。

注意 `volatile` 只保证"不被优化掉"，**不保证原子性**。这里用的都是单字节/半字/字的单次读写，才安全；如果是 `uint64_t` 或结构体就需要临界区。

#### 3.3 宏定义里的括号与向上取整

```c
#define CATCH_SETTLE_TICKS  ((CATCH_SETTLE_MS + CATCH_PERIOD_MS - 1) / CATCH_PERIOD_MS)
```

- 每个参数和整个表达式都套括号，防止 `a - b` 这种实参把优先级搞乱。
- `(x + p - 1) / p` 是整数**向上取整**的经典写法：500ms 按 12ms 一拍算，需要 42 拍（41.67 向上）。
- `#define CATCH_MOVE_COUNT(arr) ((uint8_t)(sizeof(arr) / sizeof(arr[0])))` 用 `sizeof` 求数组元素个数，加 `(uint8_t)` 收窄以匹配结构体的 `uint8_t Count`。

#### 3.4 `TickType_t` / `pdMS_TO_TICKS` / `vTaskDelayUntil`

```c
TickType_t lastWakeTime = xTaskGetTickCount();
while (1)
{
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(XUNJI_PERIOD_MS));
    ...
}
```

- `TickType_t` 是 FreeRTOS 的节拍计数类型（通常 32 位无符号）。
- `pdMS_TO_TICKS(ms)` 把毫秒换算成节拍数，避免手写 `ms / portTICK_PERIOD_MS`。
- `vTaskDelayUntil(&last, period)` 是**绝对延时**：每次按"上一次唤醒时刻 + period"来算，误差不会累积。对比 `vTaskDelay(n)` 是相对延时，周期会被执行时间拉长。所以周期性任务一律用 `vTaskDelayUntil`。
- `lastWakeTime` 必须在**循环外**初始化一次，并且是局部变量（每个任务自己有）。
- 任务函数必须永不返回，所以都是 `while (1)`。

#### 3.5 `xTaskCreate` 的 6 个参数

```c
xTaskCreate(Xunji_Task, "Xunji", 256, NULL, 3, &XunjiTaskHandler);
```

| 位置 | 实际值 | 含义 |
|---|---|---|
| 1 | `Xunji_Task` | 任务函数指针，原型固定 `void f(void *)` |
| 2 | `"Xunji"` | 任务名，调试用 |
| 3 | `256` | 栈深度，单位**字（4 字节）**，即 1KB |
| 4 | `NULL` | 传给任务函数的参数，任务里 `(void)arg` 表示不用 |
| 5 | `3` | 优先级，数字越大越高 |
| 6 | `&XunjiTaskHandler` | 输出句柄，以后可以用它 `vTaskSuspend`/`vTaskDelete` |

`TaskHandle_t` 本质是 `void *`，所以句柄类型写成 `TaskHandle_t XunjiTaskHandler;` 即可。

#### 3.6 结构体 + `const` 数组做"数据表"

```c
typedef struct { TIM_TypeDef *TIMx; uint8_t channel; uint8_t target; } CatchMove_t;
typedef struct { const CatchMove_t *Move; uint8_t Count; } CatchStage_t;

static const CatchMove_t s_Stage1[] = { { Servo1, 4, 90 } };
static const CatchStage_t s_Stages[] = {
    { s_Stage1, CATCH_MOVE_COUNT(s_Stage1) },
    ...
};
```

- `typedef struct { ... } 名字;` 省掉后面每次写 `struct`。
- 内层数组的**数组名退化成指针**，所以 `const CatchMove_t *Move` 能接住 `s_Stage1`。
- 外层 `s_Stages[]` 的每个元素记录"动作表地址 + 元素个数"，这就是用 C 语言手工实现的"变长二维表"。
- 全部 `const` 会放进 Flash（.rodata），省 RAM；`static` 限制可见性。
- `TIM_TypeDef *` 是标准库的定时器寄存器结构体指针，`Servo1` 宏就是 `TIM2`，所以数据表里能直接写 `Servo1`。

#### 3.7 指针返回 + 就地自增

```c
static uint8_t *Servo_AnglePtr(TIM_TypeDef *TIMx, uint8_t channel)
{
    if (TIMx == Servo1) return &s_Servo1Angle[channel];
    return &s_Servo2Angle[channel];
}

angle = Servo_AnglePtr(...);
(*angle)++;                 // 先解引用，再自增
Angle_SetCompare(..., (float)(*angle));
```

- 函数返回的是**数组元素的地址**，调用方通过 `*angle` 直接改原数组，不需要"读出来—改—写回去"。
- `(*angle)++` 的括号不能省：`*angle++` 会被解析成 `*(angle++)`（先移动指针再取值），语义完全不同。
- `(float)(*angle)` 是显式类型转换，因为 `Angle_SetCompare` 的第三个参数是 `float`。
- `channel` 直接当数组下标，所以数组必须开到 5（下标 0~4），下标 0 留空。

#### 3.8 字符串处理三件套

```c
if (sscanf(Serial_RxPacket, "%15[^,],%d,%d,%d", color, &x, &y, &count) == 4)
strncpy(g_ObjColor, color, sizeof(g_ObjColor) - 1);
g_ObjColor[sizeof(g_ObjColor) - 1] = '\0';
strcmp(color, "red") == 0
```

- `sscanf` 的 `%15[^,]` 表示"最多读 15 个非逗号字符"存入 `color`，后面的 `%d` 按逗号自动跳过空白？——**不会**，所以格式串里显式写了 `,`，`sscanf` 会匹配字面逗号。返回值是成功赋值的项数，`== 4` 才算格式完整。
- `%15` 的 15 是**宽度上限**，防止 `color` 只有 16 字节时被写爆，是本文件里最重要的一处安全写法。
- `strncpy` 不一定补 `'\0'`（源串长度 ≥ n 时），所以后面手动补一次。
- `strcmp` 返回 0 表示相等，不能写成 `if (strcmp(...))`。
- `strcpy` 不用是因为目标缓冲固定长、来源是串口数据，长度不可信。

#### 3.9 整数与浮点的混合运算

```c
int16_t dx = (int16_t)(g_ObjX - TRACK_TARGET_X);
cm = (uint32_t)(distance + 0.5f);
```

- `g_ObjX` 是 `int`，`dx` 用 `int16_t` 收窄，因为差值范围只可能是几百，省空间也表明意图。
- 浮点转整数是**截断**，`+0.5f` 实现四舍五入。
- `distance < 0.0f` 作为"无效值"判断，这里的 `-1.0f` 是约定值，不是误差。
- `if (dx >= -TRACK_TOL_X && dx <= TRACK_TOL_X)` 这种"绝对值判断"拆开写，比 `abs()` 省一个函数调用，也避免 `abs` 对 `int16_t` 的隐式提升问题。

#### 3.10 空语句块与空 `if`

```c
if (g_CatchRunning || g_ArmDelayTicks > 0)
{

}
else if (y < 0 || y > 300)
{

}
else
{
    ...
}
```

空 `{ }` 是合法的空复合语句，作用就是"这个条件命中时什么都不做，落到 else 继续判断"。它比 `if (!a && !b && ...)` 更易读，缺点是容易被误认为是漏写了代码。等价写法是 `continue;`，但这里必须保留后面的 `Serial_RxFlag = 0;`，所以只能用空块。

#### 3.11 `(void)arg;`

```c
static void Serial_Task(void *arg)
{
    (void)arg;
```

任务函数原型必须收一个 `void *`，但本文件不用它。`(void)arg;` 是一个"把表达式求值后丢弃"的语句，只为告诉编译器"我确实看了这个参数"，从而关掉 `unused parameter` 警告。

#### 3.12 `while (1)` + `continue` + `return` 的分工

- `continue`：跳过本次循环剩余部分，直接进入下一次 `vTaskDelayUntil`（本文件用来跳过不该执行的逻辑）。
- `return`：直接退出函数（`catch_litter()` 里大量使用，实现"闸门"式早退）。
- `while (1) { }` 放在 `vTaskStartScheduler()` 之后是**占位**：调度器正常启动后永远不返回；如果返回说明启动失败（例如堆不够），此时停在死循环里便于调试器发现。

#### 3.13 外部变量的 `extern` 声明

```c
extern char Serial_RxPacket[];
extern uint8_t Serial_RxFlag;
```

定义在 `Serial.c`，`main.c` 只声明不分配空间。`[]` 留空表示"长度由定义处决定"。这两个变量在中断里被写，所以使用时必须注意读到的是"最新完整包"——`Serial_RxFlag` 只在收齐 `\r\n` 后置 1，天然保证 `Serial_RxPacket` 已经完整。

#### 3.14 注释风格与标记

- `/* ... */`：多行说明、数据表说明。
- `//`：行尾短注释。
- `@stop`：这是**给使用者看的协议标记**，表示串口发 `@stop` 触发急停，不是 C 语法。
- 文件里 `/* 换算成20ms步数 */` 这类注释属于"宏语义说明"，和实际数值的一致性要自己维护（见第 4 节）。

---
