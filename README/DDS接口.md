# 二、DDS接口整理

# 一、前言

本文为人形使用DDS方式交互的接口说明文档，由于DDS接口是通过IDL文件定义并序列化生成的，所以本文的接口文档全部由IDL文件定义。

所有需要实时发布的话题均以\*\*rt/\*\*开头。

越疆人形因为产品形态的多样化，采用的是上下肢分控的方式，即上肢和下肢使用不同的话题控制。上肢：腰部、左臂、右臂、头部，下肢：左腿、右腿。上肢和下肢可能会不同时存在，因此像电池信息、手柄信息在上下肢状态话题中均存在，即虽然两个话题中均有定义，但实际是同一个东西。

下文中提到的底层是指管理关节、BMS、imu、手柄的程序。如果使用人员熟悉强化学习，可以使用如下接口和底层交换，然后来控制越疆机器人。

算法状态机是内置于PC1的一个程序，它在某种情况下也会对上下肢进行控制，关于算法状态机的介绍另外有文档说明，这里不再赘述。

灵巧手数组排序：右手（大拇指弯曲、食指、中指、无名指、小拇指、大拇指旋转）、左手（大拇指弯曲、食指、中指、无名指、小拇指、大拇指旋转）。

## 1.1 话题频率说明

Host 实时主循环与 CODESYS EtherCAT 任务对齐，周期约 **1200 μs（833 Hz）**。下表为各话题在工程中的典型频率；**上下肢控制话题**建议以 **833 ms** 为发布周期；Host 在实时循环内取用**最新一帧**命令下发到伺服。

| 话题                                  | 方向 | 典型频率          | 说明                                                    |
| ------------------------------------- | ---- | ----------------- | ------------------------------------------------------- |
| `rt/upper/state`                    | 状态 | ≈833 Hz         | 周期 1200 μs                                         |
| `rt/lower/state`                    | 状态 | ≈833 Hz         | 周期 1200 μs                                         |
| `rt/upper/cmd`                      | 控制 | ≈833 Hz         | 周期 1200 μs                                         |
| `rt/lower/cmd`                      | 控制 | ≈833 Hz         | 周期 1200 μs                                         |
| `rt/posctl/state`                   | 状态 | ≈833 Hz         | 周期 1200 μs                                         |
| `rt/posctl/cmd`                     | 控制 | 由发布方决定      | 事件驱动；`CART_JOG`/`RUN_TO` 建议约 10 Hz 保活 |
| `rt/main/nodes/state`               | 状态 | 5 Hz             | 周期 200 ms                                           |
| `rt/hands/state`                    | 状态 | ≈125 Hz         | 周期 8000 μs                                         |
| `rt/hands/cmd`                      | 控制 | 由发布方决定      | 事件驱动                                                |
| `rt/power/state`                    | 状态 | 5 Hz             | 周期 200 ms                                           |
| `rt/bms/info`                       | 状态 | 10 Hz            | 周期 100 ms                                           |
| `rt/amr/state`                      | 状态 | 40 Hz            | 周期 25 ms                                            |
| `rt/amr/laserscan`、`rt/amr/odom` | 状态 | 订阅后约 100 Hz | 需先 `SUBSCRIBE_LASER`                                |

## 1.2 全局关节索引与顺序

系统共 **29** 个关节轴（`MAX_AXIS=29`），Host 内部索引与 DDS 上下肢数组映射如下（见 `general_tools.h`、`dds_interface.cpp`）：

| 全局索引 | 部位 | 关节名（0 起为部位内序号）                                                                                                                                               |
| -------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0–5     | 左腿 | 0 hip\_pitch，`<br>`1 hip\_roll，`<br>`2 hip\_yaw，`<br>`3 knee，`<br>`4 ankle\_pitch，`<br>`5 ankle\_roll                                                |
| 6–11    | 右腿 | 0 hip\_pitch，`<br>`1 hip\_roll，`<br>`2 hip\_yaw，`<br>`3 knee，`<br>`4 ankle\_pitch，`<br>`5 ankle\_roll                                                |
| 12       | 腰   | torso                                                                                                                                                                     |
| 13–19   | 左臂 | 0 shoulder\_pitch，`<br>`1 shoulder\_roll，`<br>`2 shoulder\_yaw，`<br>`3 elbow\_pitch，`<br>`4 elbow\_roll，`<br>`5 wrist\_pitch，`<br>`6 wrist\_roll |
| 20–26   | 右臂 | 同左臂顺序                                                                                                                                                                |
| 27–28   | 头   | 0 head\_yaw，`<br>`1 head\_pitch                                                                                                                                      |

**DDS 数组映射：**

* `rt/lower/state` / `rt/lower/cmd` 的 `motor_state[12]` / `motor_cmd[12]`：`[0–5]` 左腿，`[6–11]` 右腿（对应全局 0–11）。
* `rt/upper/state` / `rt/upper/cmd` 的 `motor_state[17]` / `motor_cmd[17]`：`[0]` 腰，`[1–7]` 左臂，`[8–14]` 右臂，`[15–16]` 头（对应全局 12–28）。

**灵巧手** `**hands[12]**` **索引：** 右手 `[0–5]`、左手 `[6–11]`；每侧顺序为 thumb\_bend、index、middle、ring、pinky、thumb\_rotation（弧度，见 §9/§10）。

## 1.3 关节限位（位置 / 速度 / 力矩）

以下限位适用于 `**MotorCmd_**` **/** `**MotorState_**` 中的 `q`（rad）、`dq`（rad/s）、`tau`（N·m）。全局索引与 §1.2 一致。

**ATOM\_W 与 ATOM\_MAX 差异：** 仅 **左腿全局索引 0–2**（hip\_pitch / hip\_roll / hip\_yaw）位置限位不同；其余关节两机型相同。即ATOM\_W的升降关节对应左腿的0-2关节

### ATOM\_W 位置限位

| 全局索引 | 部位 | 关节            | min (rad) | max (rad) |
| -------- | ---- | --------------- | ---------- | ---------- |
| 0        | 左腿 | hip\_pitch      | \-0.9074   | 1.099      |
| 1        | 左腿 | hip\_roll       | \-0.71     | 2.356      |
| 2        | 左腿 | hip\_yaw        | \-2.33     | 0.9074     |
| 3        | 左腿 | knee            | \-0.174    | 2.53       |
| 4        | 左腿 | ankle\_pitch    | \-0.85     | 0.8        |
| 5        | 左腿 | ankle\_roll     | \-0.4      | 0.4        |
| 6        | 右腿 | hip\_pitch      | \-3.14     | 3.14       |
| 7        | 右腿 | hip\_roll       | \-3.05     | 0.56       |
| 8        | 右腿 | hip\_yaw        | \-2.75     | 2.75       |
| 9        | 右腿 | knee            | \-0.174    | 2.53       |
| 10       | 右腿 | ankle\_pitch    | \-0.85     | 0.8        |
| 11       | 右腿 | ankle\_roll     | \-0.4      | 0.4        |
| 12       | 腰   | torso           | \-3.14     | 3.14       |
| 13       | 左臂 | shoulder\_pitch | \-2.97     | 2.97       |
| 14       | 左臂 | shoulder\_roll  | \-0.43     | 3.14       |
| 15       | 左臂 | shoulder\_yaw   | \-2.97     | 2.97       |
| 16       | 左臂 | elbow\_pitch    | \-0.87     | 3.14       |
| 17       | 左臂 | elbow\_roll     | \-2.97     | 2.97       |
| 18       | 左臂 | wrist\_pitch    | \-1.57     | 1.57       |
| 19       | 左臂 | wrist\_roll     | \-1.57     | 1.57       |
| 20       | 右臂 | shoulder\_pitch | \-2.97     | 2.97       |
| 21       | 右臂 | shoulder\_roll  | \-3.14     | 0.43       |
| 22       | 右臂 | shoulder\_yaw   | \-2.97     | 2.97       |
| 23       | 右臂 | elbow\_pitch    | \-0.87     | 3.14       |
| 24       | 右臂 | elbow\_roll     | \-2.97     | 2.97       |
| 25       | 右臂 | wrist\_pitch    | \-1.57     | 1.57       |
| 26       | 右臂 | wrist\_roll     | \-1.57     | 1.57       |
| 27       | 头   | head\_yaw       | \-3.14     | 3.14       |
| 28       | 头   | head\_pitch     | \-1.57     | 0.698      |

### ATOM\_MAX 位置限位

| 全局索引 | 部位 | 关节            | min (rad) | max (rad) |
| -------- | ---- | --------------- | ---------- | ---------- |
| 0        | 左腿 | hip\_pitch      | \-3.14     | 3.14       |
| 1        | 左腿 | hip\_roll       | \-0.56     | 3.05       |
| 2        | 左腿 | hip\_yaw        | \-2.75     | 2.75       |
| 3        | 左腿 | knee            | \-0.174    | 2.53       |
| 4        | 左腿 | ankle\_pitch    | \-0.85     | 0.8        |
| 5        | 左腿 | ankle\_roll     | \-0.4      | 0.4        |
| 6        | 右腿 | hip\_pitch      | \-3.14     | 3.14       |
| 7        | 右腿 | hip\_roll       | \-3.05     | 0.56       |
| 8        | 右腿 | hip\_yaw        | \-2.75     | 2.75       |
| 9        | 右腿 | knee            | \-0.174    | 2.53       |
| 10       | 右腿 | ankle\_pitch    | \-0.85     | 0.8        |
| 11       | 右腿 | ankle\_roll     | \-0.4      | 0.4        |
| 12       | 腰   | torso           | \-3.14     | 3.14       |
| 13       | 左臂 | shoulder\_pitch | \-2.97     | 2.97       |
| 14       | 左臂 | shoulder\_roll  | \-0.43     | 3.14       |
| 15       | 左臂 | shoulder\_yaw   | \-2.97     | 2.97       |
| 16       | 左臂 | elbow\_pitch    | \-0.87     | 3.14       |
| 17       | 左臂 | elbow\_roll     | \-2.97     | 2.97       |
| 18       | 左臂 | wrist\_pitch    | \-1.57     | 1.57       |
| 19       | 左臂 | wrist\_roll     | \-1.57     | 1.57       |
| 20       | 右臂 | shoulder\_pitch | \-2.97     | 2.97       |
| 21       | 右臂 | shoulder\_roll  | \-3.14     | 0.43       |
| 22       | 右臂 | shoulder\_yaw   | \-2.97     | 2.97       |
| 23       | 右臂 | elbow\_pitch    | \-0.87     | 3.14       |
| 24       | 右臂 | elbow\_roll     | \-2.97     | 2.97       |
| 25       | 右臂 | wrist\_pitch    | \-1.57     | 1.57       |
| 26       | 右臂 | wrist\_roll     | \-1.57     | 1.57       |
| 27       | 头   | head\_yaw       | \-3.14     | 3.14       |
| 28       | 头   | head\_pitch     | \-1.57     | 0.698      |

### 速度 / 力矩额定与最大

数据来源：下肢关节模组规格表；头/臂关节模组规格表。额定转速 = 电机额定转速 ÷ 减速比 × 2π/60（头减速比 51、臂 100）；最大转速 = 电机最大转速 ÷ 减速比 × 2π/60（下肢直接取规格表关节最大转速 rad/s）。额定/最大力矩对应规格表关节额定扭矩、关节峰值扭矩（N·m）。

| 全局索引 | 部位 | 关节            | 额定速度 (rad/s) | 最大速度 (rad/s) | 额定扭矩 (N·m) | 最大扭矩 (N·m) |
| -------- | ---- | --------------- | ----------------- | ----------------- | ---------------- | ---------------- |
| 0        | 左腿 | hip\_pitch      | 11.90             | 13.80             | 56.10            | 207.76           |
| 1        | 左腿 | hip\_roll       | 8.47              | 11.90             | 69.19            | 241.42           |
| 2        | 左腿 | hip\_yaw        | 9.05              | 11.36             | 22.44            | 104.16           |
| 3        | 左腿 | knee            | 11.87             | 16.67             | 83.30            | 213.80           |
| 4        | 左腿 | ankle\_pitch    | 11.13             | 15.71             | 17.68            | 89.90            |
| 5        | 左腿 | ankle\_roll     | 11.13             | 15.71             | 17.68            | 89.90            |
| 6        | 右腿 | hip\_pitch      | 11.90             | 13.80             | 56.10            | 207.76           |
| 7        | 右腿 | hip\_roll       | 8.47              | 11.90             | 69.19            | 241.42           |
| 8        | 右腿 | hip\_yaw        | 9.05              | 11.36             | 22.44            | 104.16           |
| 9        | 右腿 | knee            | 11.87             | 16.67             | 83.30            | 213.80           |
| 10       | 右腿 | ankle\_pitch    | 11.13             | 15.71             | 17.68            | 89.90            |
| 11       | 右腿 | ankle\_roll     | 11.13             | 15.71             | 17.68            | 89.90            |
| 12       | 腰   | torso           | 5.17              | 6.49              | 39.27            | 182.28           |
| 13       | 左臂 | shoulder\_pitch | 3.14              | 4.61              | 33               | 93               |
| 14       | 左臂 | shoulder\_roll  | 3.14              | 4.50              | 22.5             | 70               |
| 15       | 左臂 | shoulder\_yaw   | 3.14              | 4.50              | 22.5             | 70               |
| 16       | 左臂 | elbow\_pitch    | 3.14              | 4.50              | 22.5             | 70               |
| 17       | 左臂 | elbow\_roll     | 3.14              | 5.13              | 7.5              | 34               |
| 18       | 左臂 | wrist\_pitch    | 3.14              | 5.13              | 7.5              | 34               |
| 19       | 左臂 | wrist\_roll     | 3.14              | 5.13              | 7.5              | 34               |
| 20       | 右臂 | shoulder\_pitch | 3.14              | 4.61              | 33               | 93               |
| 21       | 右臂 | shoulder\_roll  | 3.14              | 4.50              | 22.5             | 70               |
| 22       | 右臂 | shoulder\_yaw   | 3.14              | 4.50              | 22.5             | 70               |
| 23       | 右臂 | elbow\_pitch    | 3.14              | 4.50              | 22.5             | 70               |
| 24       | 右臂 | elbow\_roll     | 3.14              | 5.13              | 7.5              | 34               |
| 25       | 右臂 | wrist\_pitch    | 3.14              | 5.13              | 7.5              | 34               |
| 26       | 右臂 | wrist\_roll     | 3.14              | 5.13              | 7.5              | 34               |
| 27       | 头   | head\_yaw       | 6.16              | 11.71             | 4                | 12               |
| 28       | 头   | head\_pitch     | 6.16              | 11.71             | 4                | 12               |

# 二、接口说明

共用的子IDL文件

1. bms信息：bms\_state.idl

   ```shell
   cat bms_state.idl
   struct BmsState_
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct BmsState_
               {
                   uint16  bms_state;                     /*BMS状态*/
                   uint16  afe_state;                     /*AFE芯片状态*/
                   uint32  bms_alarms;                    /*BMS故障码*/
                   uint16  battery_level;                 /*电池电量百分比*/
                   uint16  battery_health;                /*电池健康度*/
                   uint16  pcb_board_temp;                 /*PCB板温度*/
                   uint16  afe_chip_temp;                  /*AFE芯片温度*/
                   uint16  battery_now_current;            /*电池包当前电流*/
                   uint16  cells_voltage[16];             /*16个电芯电压*/
                   uint16  battery_pack_current_voltage;    /*电池包电压*/
                   uint16  battery_pack_io_voltage;         /*电池包放电/充电接口的电压*/
                   uint32  bms_work_time;                  /*BMS运行时间*/
                   uint16  bms_hardware_version;           /*BMS硬件版本号*/
                   uint16  bms_software_version;           /*BMS软件版本号*/
                   uint16  heartbeat;                    /*心跳*/
               };
           };
       };
   };
   ```

   | 字段                             | 原始类型 | 物理单位 |
   | -------------------------------- | -------- | -------- |
   | `battery_level`                | uint16   | %        |
   | `battery_health`               | uint16   | %        |
   | `pcb_board_temp`               | uint16   | ℃       |
   | `afe_chip_temp`                | uint16   | ℃       |
   | `battery_now_current`          | uint16   | mA       |
   | `cells_voltage[i]`             | uint16   | V        |
   | `battery_pack_current_voltage` | uint16   | V        |
   | `battery_pack_io_voltage`      | uint16   | V        |
   | `bms_work_time`                | uint32   | s        |
2. 关节信息：motor\_state.idl

   ```shell
   cat motor_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorState_ {
                   uint8 mode;                  // 模式
                   float q;                     // 角位置
                   float dq;                    // 角速度
                   float ddq;                   // 角加速度
                   float tau_est;               // 估计的力矩
                   float q_raw;                 // 原始角位置
                   float dq_raw;                // 原始角速度
                   float ddq_raw;               // 原始角加速度
                   uint8 mcu_temp;               // 伺服控制板温度
                   uint8 mos_temp;               // mos管温度
                   uint8 motor_temp;             // 电机温度
                   uint8 bus_voltage;            // 母线电压数据
               };
           };
       };
   };

   ```

   | 字段                                         | 单位  | 说明                                                                                 |
   | -------------------------------------------- | ----- | ------------------------------------------------------------------------------------ |
   | `mode`                                     | —    | 伺服 CiA402 显示模式：`11`\=MIT，`8`\=位控，`6`\=回零等（`byDisplayMode`） |
   | `q`, `q_raw`                            | rad   | 关节角位置                                                                           |
   | `dq`, `dq_raw`                          | rad/s | 关节角速度                                                                           |
   | `ddq`, `ddq_raw`                        | —    | 当前实现未填充有效角加速度；踝部 `ddq_raw` 为内部调试复用                         |
   | `tau_est`                                  | N·m  | 估计关节力矩                                                                         |
   | `mcu_temp`, `mos_temp`, `motor_temp` | ℃    | 伺服板 / MOS / 电机温度                                                          |
   | `bus_voltage`                              | V     | 母线电压                                                                             |
3. 关节指令:motor\_cmd.idl

   ```shell
   cat motor_cmd.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct MotorCmd_ {
                   uint8 mode;                        // 模式
                   float q;                           // 位置
                   float dq;                          // 速度
                   float tau;                         // 力矩
                   float kp;                          // 比例增益
                   float kd;                          // 微分增益
               };
           };
       };
   };

   ```

   | 字段    | 单位  | 建议范围                   | 说明       |
   | ------- | ----- | -------------------------- | ---------- |
   | `q`   | rad   | §1.3 各全局索引 min~max | 目标位置   |
   | `dq`  | rad/s | 腿/腰 ±20，臂/头 ±99   | 目标速度   |
   | `tau` | N·m  | ±1000                     | 前馈力矩   |
   | `kp`  | —    | \>0                        | 位置环增益 |
   | `kd`  | —    | ≥0                        | 速度环增益 |

   请勿传入 NaN/Inf；位置/速度/力矩超限可通过 `rt/main/nodes/state` 中对应 `*_err_code` 字段查询。
4. imu信息：imu\_state.idl

   ```shell
   cat imu_state.idl
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct IMUState_ {
                   float quaternion[4];        // 四维四元数
                   float gyroscope[3];         // 三维陀螺仪数据，单位deg/s
                   float accelerometer[3];     // 三维加速度计数据，单位m/s2
                   float rpy[3];               // 三维欧拉角（滚转、俯仰、偏航）,单位deg
                   uint8 temperature;          // 温度传感器数据,单位℃
               };
           };
       };
   };
   ```
5. 下文中提到的wireless\_remote说明：

   对应键按下时有值，松开时，恢复为0。

   ```shell
   wireless_remote[2]，从高位到低位依次对应键：'', '', 'LT', 'RT', 'SELECT', 'START', 'LB', 'RB';
   wireless_remote[3]，从高位到低位依次对应键：'LEFT', 'DOWN', 'RIGHT', 'UP', 'Y', 'X', 'B', 'A';
   wireless_remote[4->7]，存放的是LX键转成[-1,0]之间的浮点值value，wireless_remote[4]是value低位；
   wireless_remote[8->11]，存放的是RX键的值[0,1]之间的浮点值value，wireless_remote[8]是value低位；
   wireless_remote[12->15]，存放的是RY键的值[-1,0]之间的浮点值value，wireless_remote[12]是value低位；
   wireless_remote[16->19]，存放的是LY键的值[0,1]之间的浮点值value，wireless_remote[16]是value低位。
   ```
6. 底盘状态:amr\_state.idl

   1. AMRState\_为整个底盘的主要状态

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum NavigationStatus_ {
                UNKNOWN,            // 未知
                QUEUING,            // 排队
                RUNNING,            // 运行中
                COMPLETED,          // 完成
                FAILED,             // 失败
                PAUSED,             // 暂停
                CANCELED,           // 取消
                WAITING_CONFIRM,    // 等待确认
                IDLE,               // 空闲
                STOPPED             // 停止状态
            };// 枚举仅作navigation_status的数值参考

            enum DeviceStatus_ {
                DEVUNKNOWN,        // 未知
                DEVIDLE,           // 小车空闲
                TASKING,        // 任务中
                ERROR,          // 故障中
                OFFLINE,        // 离线
                INIT,           // 设备初始化
                CHARGING,       // 充电中
                UPGRADE         // 升级中
            }; // 枚举仅作device_status的数值参考
            @extensibility(FINAL)
            struct AMREventStatus_{
                boolean emergency_stop_pressed;             // 急停按钮被按下
                boolean enable_pressed;                     // 使能键被按下
                boolean path_blocked;                        // 路径被挡住
                boolean low_battery;                         // 电池电量过低
                boolean obstacle_detected;                   // 被障碍物挡住
            
            };
        
            @extensibility(FINAL)
            struct AMRBasicStatus_{
                float battery_level;        // 电池电量百分比
                float battery_voltage;      // 电池电压
                float battery_current;      // 电池电流
                uint16 heartbeat;          // 心跳值
            };

            @extensibility(FINAL)
            struct AMRState_ {
                DeviceStatus_ device_status;                         // 设备状态
                NavigationStatus_ navigation_status;                    // 导航状态 参考上述enum，用于判断任务是否完成
                AMRBasicStatus_ basic_status;                // 基础状态
                float position[3];                          // 当前位置 {x,y,yaw}
                AMREventStatus_ amr_event;                   // 底盘相关事件 
                uint32 error_code[32];                        // 错误码
                uint32 task_id;                           // 任务ID
                uint32 work_mode;                         // 机器人工作模式 枚举如右： 0：任务模式 || 3：遥控模式 || 4:检修模式
            };



        };
    };
};
```

| 字段                              | 单位 |
| --------------------------------- | ---- |
| `basic_status.battery_level`    | %    |
| `basic_status.battery_voltage`  | V    |
| `basic_status.battery_current`  | A    |
| `basic_status.heartbeat`        | —   |
| `position[0]`, `position[1]` | m    |
| `position[2]`                   | rad  |

## 1 切换底层状态

1. 功能说明：涉及到安全防护的原因，如果需要控制关节，要使用该接口将底层状态设置为非零力矩模式，否则底层不将控制指令下发到关节。
2. 话题名称：rt/set/fsm/id
3. **注意，此接口不对外开放！**
4. IDL文件内容：

```shell
module dobot_atom{
    module msg{
        module dds_{
            // 算法/策略侧执行状态
            enum WorkingState {
                HIDLE,        // 空闲
                HTASKING      // 任务中(如开启了运控,上肢控制,调试模式等,则状态为HTASKING)
            }; //这里加H前缀的原因(H表示Human)，是在amr_state.idl定义了IDLE,TASKIG,这里如果再同样命名为IDLE，会导致c++编译报错

            @extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id;               // 对算法状态机
                string current_action;   // 算法当前正在执行的动作，空表示没有动作
                WorkingState state; // HIDLE | HTASKING
            };
        };
    };
};





```

## 2 切换上肢控制

1. 功能说明：避免外部（用户或者遥操作）和算法状态机同时控制上肢，所以需要增加该接口，如果外部包括PC2要对上肢进行控制时，要打开该开关。
2. 话题名称：rt/switch/upper/control
3. IDL文件内容：

```shell
cat switch_upper_control.idl
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SwitchUpperControl_ {
                boolean flag;  /* true: 上肢有控制，false: 上肢无控制*/
            };
        };
    };
};
```

## 3 上肢状态

1. 功能说明：上肢底层信息。
2. 话题名称：rt/upper/state
3. **反馈频率：** ≈833 Hz（周期 1200 μs，见 §1.1）。
4. **包含关节：** `motor_state[17]` 对应全局索引 12–28：`[0]` 腰，`[1–7]` 左臂（肩→腕 7 轴），`[8–14]` 右臂，`[15–16]` 头。另含 `bms_state`、`wireless_remote[40]`、`fsm_id`、`is_upper_control`、`robot_type`。
5. IDL文件内容：

```shell

#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct UpperState_ {
                char robot_type[16];           /*对应产品定义的类型*/
                boolean is_upper_control;      /* true: 上肢有控制，false: 上肢无控制*/
                uint16 fsm_id;                 /*对应算法状态机*/
                MotorState_ motor_state[17];
                BmsState_ bms_state;
                octet wireless_remote[40];
                uint32 reserve;               /*越疆保留*/
            };
        };
    };
};

```

## 4 上肢控制

1. 功能说明：下肢状态信息。
2. 话题名称：rt/upper/cmd
3. **反馈频率：** ≈833 Hz（周期 1200 μs，见 §1.1）。
4. **包含关节：** `motor_state[12]`，话题下标 `k` 即全局索引 `k`（0–11）：0–5 左腿，6–11 右腿。另含 `imu_state`、`bms_state`、`wireless_remote[40]`、`fsm_id`。
5. IDL文件内容：

```shell
cat upper_cmd.idl
#include "motor_cmd.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct UpperCmd_ {
                MotorCmd_ motor_cmd[17];           // 电机命令，包含 17 个 MotorCmd_ 结构体
            };
        };
    };
};

```

## 5 下肢状态

1. 功能说明：下肢控制指令。
2. 话题名称：rt/lower/state
3. **控制频率：** 建议发布周期 **833 ms（2 Hz）**（见 §1.1）。
4. **关节顺序：** `motor_cmd[12]` 与 §5 一致，话题下标 `k` 即全局索引 `k`：0–5 左腿（全局0 hip\_pitch 起），6–11 右腿。字段单位见 `MotorCmd_`（§2.3）；位置限位见 §1.3。
5. IDL文件内容：

```c++
cat lower_state.idl
#include "imu_state.idl"
#include "motor_state.idl"
#include "bms_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct LowerState_ {
                uint16 fsm_id;          /*对应算法状态机*/
                IMUState_ imu_state;
                MotorState_ motor_state[12];
                BmsState_ bms_state;
                octet wireless_remote[40];
                uint32 reserve;               /*越疆保留*/
            };
        };
    };
};
```

## 6 下肢控制

1. 功能说明：下肢控制指令。
2. 话题名称：rt/lower/cmd
3. IDL文件内容：

```shell
cat lower_cmd.idl
#include "motor_cmd.idl"

struct LowerCmd_ {
    MotorCmd_ motor_cmd[12];           // 电机命令，包含 12 个 MotorCmd_ 结构体
};

```

## 7 关节、can板状态

1. 功能说明：主要是提供给用户辅助调试使用的。
2. 话题名称：rt/main/nodes/state
3. **反馈频率：** 5 Hz（200 ms）。覆盖全身 29 轴调试信息（左/右腿各 6、腰、左/右臂各 7、头 2）及 2 路 ECAT2CAN 从站。
4. IDL文件内容：

   ```c++
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct AxisStateInfo_
               {
                   uint8  servo_state;
                   uint16 error_code;  //电机报警错误码
                   uint16 warn_code;  // 电机警告错误码
                   int32  pos_err_code;
                   int32  vel_err_code;
                   int32  torque_err_code;
                   uint8  node_state;
                   uint8  display_op_mode;
                   boolean is_virtual;   /* default: false */
                   uint8  mcu_temp;
                   uint8  mos_temp;
                   uint8  motor_temp;
                   uint8  bus_voltage;
                   uint16 software_version;
               };

               @extensibility(FINAL)
               struct EcatSlaveInfo_
               {
                   boolean is_virtual;
                   uint8   slave_state;
                   uint16  error_code;
                   uint16  software_version;
               };

               @extensibility(FINAL)
               struct MainNodesState_ {
                   AxisStateInfo_ left_leg[6];
                   AxisStateInfo_ right_leg[6];
                   AxisStateInfo_ waist;
                   AxisStateInfo_ left_arm[7];
                   AxisStateInfo_ right_arm[7];
                   AxisStateInfo_ head[2];
                   EcatSlaveInfo_ ecat2can[2];
                   /* AxisStateInfo left_hand[6]; */
                   /* AxisStateInfo right_hand[6]; */
                   /* uint16 power_state[8]; */
               };
           };
       };
   };


   ```

   * node\_state枚举：

     * 0: Initialisation
     * 1: Disconnecting
     * 2: Connecting or Preparing
     * 4: Stopped
     * 5: Operational
     * 7: Pre\_operational
     * 其他: UNKNOWN
   * servo\_state枚举:    

     * 0: disable
     * 1: error
     * 3: enable
     * 其他: UNKNOWN
   * pos\_err\_code: 位置是否超限，超限则有数据
   * vel\_err\_code: 速度超限
   * torque\_err\_code: 扭矩是否超限

## 8 清错

1. 功能说明：仅清除底层的错误。
2. 话题名称：rt/clear/errors
3. IDL文件内容：

```shell
cat clear_errors.idl
struct ClearErrors_ {
    int32 msg_id;
};

```

## 9 灵巧手状态

1. 功能说明：仅灵巧手各手指角度信息。
2. 话题名称：rt/hands/state
3. **反馈频率：** ≈125 Hz（8000 μs 周期，见 §1.1）。
4. **角度单位：** `HandsState_.hands[i].q` / `.dq` 为 **rad** / **rad/s**。大寰夹爪使用 `DHGripperState_`（见 §18），`realtime_position` 为 **0~1** 开口比。
5. **数组顺序：** `[0–5]` 右手、`[6–11]` 左手；每侧：thumb\_bend、index、middle、ring、pinky、thumb\_rotation。
6. IDL文件内容：

```shell
#include "motor_state.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct HandsState_
            {
                MotorState_ hands[12];
            };
        };
    };
};



```

## 10 灵巧手控制

1. 功能说明：仅灵巧手各手指角度控制。
2. 话题名称：rt/hands/cmd
3. **控制频率：** 由发布方决定（事件驱动）。
4. **取值范围与单位：** `HandsCmd_.hands[i]` 复用 `MotorCmd_`：

   * `q`：**rad**，目标关节角（各指有效区间因机型而异，傲意约 **0.04–3.1 rad**）；

     * `dq`：**rad/s**，手指角速度；
     * 大寰夹爪：使用 `**DHGripperCmd_**`（§17），`position` 填 **0~1**。
5. **数组顺序：** 同 §9。
6. IDL文件内容：

```shell
cat hands_cmd.idl
#include "motor_cmd.idl"

module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct HandsCmd_
            { 
                MotorCmd_ hands[12];
            };
        };
    };
};

```

## 11 轮式人形底盘状态

1. 功能说明：主要包含底盘的各个状态信息
2. 话题名称：rt/amr/state
3. **反馈频率：** 40 Hz（25 ms）。
4. **单位说明：** 见 §2.6 `AMRBasicStatus_`、`position[]` 与 `velocity`。
5. IDL文件内容：

   ```shell
   module dobot_atom{
       module msg{
           module dds_{
               enum NavigationStatus_ {
                   UNKNOWN,            // 未知
                   QUEUING,            // 排队
                   RUNNING,            // 运行中
                   COMPLETED,          // 完成
                   FAILED,             // 失败
                   PAUSED,             // 暂停
                   CANCELED,           // 取消
                   WAITING_CONFIRM,    // 等待确认
                   IDLE,               // 空闲
                   STOPPED             // 停止状态
               };// 枚举仅作navigation_status的数值参考

               enum DeviceStatus_ {
                   DEVUNKNOWN,        // 未知
                   DEVIDLE,           // 小车空闲
                   TASKING,        // 任务中
                   ERROR,          // 故障中
                   OFFLINE,        // 离线
                   INIT,           // 设备初始化
                   CHARGING,       // 充电中
                   UPGRADE         // 升级中
               }; // 枚举仅作device_status的数值参考
               @extensibility(FINAL)
               struct AMREventStatus_{
                   boolean emergency_stop_pressed;             // 急停按钮被按下
                   boolean enable_pressed;                     // 使能键被按下
                   boolean path_blocked;                        // 路径被挡住
                   boolean low_battery;                         // 电池电量过低
                   boolean obstacle_detected;                   // 被障碍物挡住

               };

               @extensibility(FINAL)
               struct AMRBasicStatus_{
                   float battery_level;        // 电池电量百分比
                   float battery_voltage;      // 电池电压
                   float battery_current;      // 电池电流
                   uint16 heartbeat;          // 心跳值
               };

               @extensibility(FINAL)
               struct Velocity_ {
                   float linear_vel;   // 线速度（米/秒）
                   float angular_vel;  // 角速度（弧度/秒）
               };

               @extensibility(FINAL)
               struct AMRState_ {
                   DeviceStatus_ device_status;                         // 设备状态
                   NavigationStatus_ navigation_status;                    // 导航状态 参考上述enum，用于判断任务是否完成
                   AMRBasicStatus_ basic_status;                // 基础状态
                   float position[3];                          // 当前位置 {x,y,yaw}
                   Velocity_ velocity;                      // 融合速度（线速度、角速度见 Velocity_ 字段说明）
                   AMREventStatus_ amr_event;                   // 底盘相关事件 
                   uint32 error_code[32];                        // 错误码
                   uint32 task_id;                           // 任务ID
                   uint32 work_mode;                         // 机器人工作模式 枚举如右： 0：任务模式 || 3：遥控模式 || 4:检修模式
                   uint32 relocate_state;                    //底盘重定位状态 枚举：0：没有重定位 || 1：重定位中 || 2：重定位完成 || 3：重定位失败
                   uint32 map_switch_state;                  //地图切换状态 枚举：0:没有切换地图 || 1：切换地图中 || 2：切换完成 || 3：切换失败
               };



           };
       };
   };
   ```
6. **重定位 / 切图状态字段：** `relocate_state`、`map_switch_state` 取值见 §2.6 表格；分别配合 `RELOCATE`、`SWITCH_MAP` 命令使用（见 §12.5）。

## 12 轮式人形底盘控制

1. 功能说明：向轮式人形机器人底盘下发控制指令。
2. 话题名称：rt/amr/cmd
3. IDL文件内容：

   ```shell
     module dobot_atom{
         module msg{
             module dds_{
                 enum AMRCommandType {
                     CANCEL_TASK,      // 取消任务
                     PAUSE_TASK,       // 暂停任务
                     RESUME_TASK,      // 恢复任务
                     MOVE_TO_TAG,      // 移动到标记点, target_id必填，theta选填，theta为绝对角度(度),如果为0或不填则不指定角度
                     MOVE_TO_CHARGE,   // 回桩充电（启动自动回充流程，见下文）
                     REMOTE_CONTROL,    // 遥控
                     ROTATE,            // 原地旋转 theta必填，为相对角度(度)，相对当前底盘航向
                     START_REMOTE,      // 开始遥控
                     STOP_REMOTE,       // 关闭遥控
                     SUBSCRIBE_LASER,     // 订阅底盘激光雷达数据
                     UNSUBSCRIBE_LASER,   // 取消订阅底盘激光雷达数据
                     START_MAPPING,       // 底盘开始扫图
                     SAVE_MAP,            // 保存当前扫的图在本地
                     STOP_MAPPING,        // 结束扫图
                     SET_VEL,             // 设置速度上限（同时设置线速度和角速度，空载和负载）
                     SET_ACCEL,           // 设置加速度（同时设置线性和角度的加速度，空载和负载）
                     SET_DECEL,           // 设置减速度（同时设置线性和角度的减速度，空载和负载）
                     CANCEL_CHARGE,       // 取消充电
                     SWITCH_MAP,          // 切换地图, target_id必填，为地图编号
                     RELOCATE             // 重定位, target_id必填为节点编号, theta选填为目标角度(度)
                 };

                 @extensibility(FINAL)
                 struct AMRCommand_ {
                     AMRCommandType command_type;    // 命令类型,参考上述AMRCommandType
                     uint32 target_id;             // 目标ID（MOVE_TO_TAG/SWITCH_MAP/RELOCATE 等）
                     float linear_vel;             // 线速度 ,m/s（REMOTE_CONTROL/SET_VEL/SET_ACCEL/SET_DECEL）
                     float angular_vel;            //角速度  ,rad/s
                     uint32 command_id;            // 命令ID（用于跟踪）
                     uint64 timestamp;             // 时间戳
                     float theta;                // 角度(度): MOVE_TO_TAG/RELOCATE 为绝对角, ROTATE 为相对角
                 };
             };

         };
     };

   ```

   1. **命令类型与字段：**

   | 命令                                           | 主要字段                         | 说明                                                                                                               |
   | ---------------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
   | CANCEL\_TASK                                   | —                               | 取消当前行走任务；**仅当** `navigation_status == RUNNING` 时有效                                       |
   | PAUSE\_TASK                                    | —                               | 暂停当前行走任务；**仅当** `navigation_status == RUNNING` 时有效                                       |
   | RESUME\_TASK                                   | —                               | 恢复被暂停的行走任务                                                                                               |
   | MOVE\_TO\_TAG                                  | `target_id`, `theta`(选填)  | 导航到拓扑标记点；`theta` 为到点绝对朝向（**度**，0,360；负值按 0，360 取模）                           |
   | MOVE\_TO\_CHARGE                               | —                               | 启动自动回充流程（见 §12.4）；非立即强制下发单次回桩                                                             |
   | CANCEL\_CHARGE                                 | —                               | 取消充电并离开充电桩；**须先**通过 `MOVE_TO_CHARGE` 启动过自动回充，否则无效（含人工推桩进入充电的场景） |
   | REMOTE\_CONTROL                                | `linear_vel`, `angular_vel` | 遥控速度；须先 `START_REMOTE`；若已 `SET_VEL` 会按比例限幅                                                   |
   | ROTATE                                         | `theta`                        | 相对当前航向原地旋转（**度**）                                                                               |
   | START\_REMOTE / STOP\_REMOTE                 | —                               | 进入/退出遥控模式（`work_mode` 3/0）；退出前须停遥控                                                            |
   | SUBSCRIBE\_LASER / UNSUBSCRIBE\_LASER        | —                               | 订阅/取消订阅底盘激光雷达数据                                                                                      |
   | START\_MAPPING / SAVE\_MAP / STOP\_MAPPING | —                               | 建图控制                                                                                                           |
   | SET\_VEL                                       | `linear_vel`, `angular_vel` | 空载/负载线速度与角速度上限（m/s、rad/s）                                                                          |
   | SET\_ACCEL                                     | `linear_vel`, `angular_vel` | 线/角加速度（m/s²、rad/s²），复用同一 struct 字段                                                              |
   | SET\_DECEL                                     | `linear_vel`, `angular_vel` | 线/角减速度，字段同上                                                                                              |
   | SWITCH\_MAP                                    | `target_id`                    | 切换地图；若当前地图 ID 已一致则直接成功                                                                         |
   | RELOCATE                                       | `target_id`, `theta`(选填)  | 重定位到节点；`theta` 为目标朝向（**度**，归一化到 0,360）                                               |

   **通用前置条件：** 底盘在线且处于可接收命令的状态。自主导航类命令要求底盘当前可规划新任务。机器人上肢处于阻尼模式时，`MOVE_TO_TAG` 会被拒绝。
4. **对于遥控功能**

   1. 开始遥控前，要发送 `START_REMOTE`
   2. 然后通过 `REMOTE_CONTROL`，根据 `linear_vel`（m/s）和 `angular_vel`（rad/s）下发；无需高频重复发送，保持最近一次指令即可。若已通过 `SET_VEL` 配置上限，超出部分会被等比例缩小
   3. 不再需要遥控时请发送 `STOP_REMOTE`，否则底盘无法接收新任务
5. **对于导航到标记点（MOVE\_TO\_TAG）：**

   1. 参数

      1. `target_id`（必填）：拓扑节点/标记点编号
      2. `theta`（选填）：到点绝对朝向，单位**度**；不填或为 0 表示不指定朝向；负值按 0 处理
   2. 如何判断当前任务已完成？

      1. 订阅 `rt/amr/state`：`navigation_status` 变为 2（RUNNING）表示执行中，变为 3（COMPLETED）表示完成；失败为 4（FAILED）
   3. 如何判断任务已经下发成功？

      1. 订阅 `rt/amr/state`：`device_status` 变为 2（TASKING）表示底盘已接收并开始执行任务
   4. 是否可以在上一次任务未完成时就下发下一次任务？

      1. 不建议；不会自动取消上一任务，通常会等上一任务结束后再执行新任务
      2. 若需抢占，请先 `CANCEL_TASK`，再下发新导航命令
   5. **暂停 / 取消：** 仅当 `navigation_status == RUNNING` 时 `PAUSE_TASK` / `CANCEL_TASK` 才会生效
6. **对于自动回充（MOVE\_TO\_CHARGE）：**

   发送 `MOVE_TO_CHARGE` 后，控制器启动自动回充流程，根据实时电量与底盘状态决定是否、何时下发回桩任务；**不是**每条 DDS 命令都等价于一次立即回桩。

   1. **电量阈值：** 低电阈值与严重低电阈值由机器人侧配置（典型默认值约 20% / 5%）。电量高于低电阈值且未在充电时，自动回充流程会结束（视为无需回充）
   2. **回桩下发时机：** 底盘空闲（`device_status == DEVIDLE`）且电量 ≤ 低电阈值时，才会尝试下发回桩；同一轮自动回充中通常只尝试一次。若下发后约 1 s 内 `navigation_status == FAILED`，可视为找不到充电点或回充失败
   3. **回充成功：** `device_status == CHARGING` 表示**回充完成**（已对接充电桩），可通过订阅 `rt/amr/state` 判断
   4. **离开充电桩：** 回充完成并充上电后，需发送 `CANCEL_CHARGE` 才能退出充电桩。**必须先下发过** `MOVE_TO_CHARGE` **启动自动回充**，`CANCEL_CHARGE` 才会生效；若为人工推桩等方式进入充电、未走自动回充流程，此时下发 `CANCEL_CHARGE` **无效**
   5. **流程中断（未成功上桩）：**

      1. 回桩途中收到其他导航任务（如 `MOVE_TO_TAG`）→ 自动回充流程中断，需再次发送 `MOVE_TO_CHARGE` 才能重新进入
      2. 回桩途中机器人上半身开始运动任务 → 取消底盘回桩并中断自动回充
      3. 电量 ≤ 严重低电阈值 → 取消底盘导航任务，并触发上半身安全保护（阻尼）
7. **对于切换地图（SWITCH\_MAP）与重定位（RELOCATE）：**

   下发命令后，通过订阅 `rt/amr/state` 中的 `map_switch_state`、`relocate_state` 判断进度

   1. `SWITCH_MAP`

      1. 参数：`target_id` 为目标地图 ID；若与当前地图 ID 一致，命令侧可直接视为成功
      2. 状态判断：读取 `map_switch_state` — 0：没有切换；1：切换中；2：切换完成；3：切换失败
   2. `RELOCATE`

      1. 参数：`target_id` 为节点编号；`theta` 为可选目标朝向（度）
      2. 状态判断：读取 `relocate_state` — 0：没有重定位；1：重定位中；2：重定位完成；3：重定位失败

## 13 摇杆控制

1. ~~功能说明：主要用于将app发送的摇杆的数据，转发给到算法使用~~
2. ~~话题名称：rt/remote/control~~
3. ~~注意，此话题不对外开放~~
4. ~~IDL文件内容：~~

   ```shell
   module dobot_atom{
       module msg{
           module dds_{
               @extensibility(FINAL)
               struct JoystickValue_ {
                   float x;
                   float y;  
               };

               @extensibility(FINAL)
               struct RemoteControl_ {
                   JoystickValue_ btn_move;    // 左旋钮键值
                   JoystickValue_ btn_turn;    // 右旋钮键值
               };
           };
       };
   };


   ```
5. ~~如上，其中，一个摇杆有两个旋钮分别是左旋钮和右旋钮，分别存储在btn\_move和btn\_turn，~~
6. ~~以左旋钮为例，当旋钮往右拨动时，x值增加，反之则减，当往上拨动时，y值增加，反之则减~~
7. ~~JoystickValue\_阈值区间： \[-1,1\]~~

## 14 急停状态

1. 功能说明：整个嵌入式系统中的急停信息。
2. 话题名称：rt/emergency/state
3. IDL文件内容：

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct EmergencyState_ {
                boolean soft_emergency_triggered;   // 软紧急停止 (app触发)
                boolean hard_emergency_triggered;   // 硬紧急停止 (用户板触发)
                boolean amr_emergency_triggered;    // AMR紧急停止 (轮式底盘触发)
            };
        };
    };
};

```

## 15 状态机状态

1. 功能说明：由算法发布的状态机的信息
2. 话题名称：rt/set/fsm/id
3. IDL文件内容：

   * id : 当前算法状态机装固态
   * current\_action: string，当用户通过app或者摇杆执行HandsUp时，算法这个字段设为“HandsUp",当动作执行完，current\_action置为空

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct SetFsmId_ {
                uint16 id;         // 对算法状态机
                char current_action; // 算法当前正在执行的动作，空表示没有动作
            };
        };
    };
};

```

## 15 AMR激光雷达数据

1. 功能说明：当通过[**amr\_cmd**](https://alidocs.dingtalk.com/i/nodes/KGZLxjv9VG37o9N6I7N2zj4XV6EDybno?utm_scene=team_space&utm_medium=main_vertical&utm_source=search)发送一个++**订阅激光雷达**++的命令之后，线程开始发布轮式人形底盘的激光雷达数据和里程计数据
2. 话题名称：rt/amr/laserscan
3. IDL文件内容：

   * timestamp : 发布激光雷达的时间戳
   * lidar:激光雷达信息，最多 1024 个元素，表示每个角度的激光测距，单位 mm

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct LaserScan_ {
                uint64 timestamp;            // 时间戳（ms）
                int32 lidar[1024];          // 激光数据（最多1024点，未满补0）,单位mm
            };
        };
    };
};



```

## 16 AMR里程计数据

1. 功能说明：当通过**amr\_cmd**发送一个++**订阅激光雷达**++的命令之后，线程开始发布轮式人形底盘的激光雷达数据和里程计数据
2. 话题名称：rt/amr/odom
3. IDL文件内容：

   * timestamp : 发布激光雷达的时间戳

```shell
module dobot_atom{
    module msg{
        module dds_{
            @extensibility(FINAL)
            struct Odom_ {
                uint64 timestamp;    // 时间戳（ms）
                int32 x;            // 里程计X坐标，单位mm
                int32 y;            // 里程计Y坐标，单位mm
                int32 angle;        // 里程计角度，单位0.001度
            };
        };
    };
};

```

## 17 大寰夹爪命令

a. **功能说明**：向左右大寰夹爪下发初始化、目标开口位置与夹持力。Host 订阅后通过 Modbus 写夹爪寄存器。

b. **话题名称**：`rt/hands/cmd`

c. **IDL 文件内容**（`dh_gripper_cmd.idl`）：

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum DHGripperInitCmd {
                DH_INIT_NONE,        // 不触发初始化
                DH_INIT_HOMING      // 回零位（找单向位置）
            };

            @extensibility(FINAL)
            struct DHGripperSingleCmd_ {
                DHGripperInitCmd init_cmd;
                float position;  // 0~1，1.0=完全张开（内部映射寄存器 0~1000）
                float force;
            };

            @extensibility(FINAL)
            struct DHGripperCmd_ {
                DHGripperSingleCmd_ right;
                DHGripperSingleCmd_ left;
            };
        };
    };
};
```

i. 参考上述 IDL，单侧夹爪主要字段：

1. `init_cmd`：`DH_INIT_HOMING` 触发回零初始化；`DH_INIT_NONE` 不触发
2. `position`：目标开口 **0~1**
3. `force`：夹持力，常用约 **20~100**（机载默认 20）

ii. 推荐使用顺序：

1. 发布 `init_cmd = DH_INIT_HOMING`（左/右可分别配置）
2. 订阅 `rt/hands/state`，等待两侧 `init_result_state == DH_INIT_SUCCESS`
3. 将 `init_cmd` 置为 `DH_INIT_NONE`，再按需发布 `position` / `force`
4. 未初始化成功前，位置命令会被 Host 忽略；初始化超时约 **10 s**，需重新 HOMING

## 18 大寰夹爪状态

a. **功能说明**：Host 周期发布左右夹爪初始化结果、夹持状态、实时开口与力反馈。

b. **话题名称**：`rt/hands/state`

c. **IDL 文件内容**（`dh_gripper_state.idl`）：

```shell
module dobot_atom{
    module msg{
        module dds_{
            enum DHGripperInitResultState {
                DH_INIT_NOT_DONE,
                DH_INIT_SUCCESS,
                DH_INIT_EXECUTING
            };

            enum DHGripperGripState {
                DH_GRIP_MOVING,
                DH_GRIP_IN_PLACE,
                DH_GRIP_HOLDING_OBJECT,
                DH_GRIP_OBJECT_DROPPED
            };

            @extensibility(FINAL)
            struct DHGripperSingleState_ {
                DHGripperInitResultState init_result_state;
                DHGripperGripState grip_state;
                float realtime_position;  // 0~1
                float get_force_value;
            };

            @extensibility(FINAL)
            struct DHGripperState_ {
                DHGripperSingleState_ right;
                DHGripperSingleState_ left;
            };
        };
    };
};
```

i. `init_result_state`：`0` 未初始化，`1` 成功，`2` 执行中

ii. `grip_state`：`0` 运动中，`1` 到位，`2` 夹住物体，`3` 物体掉落

## 19 位置控制关节位控命令

a. **功能说明**：切换 MIT / 位控应用模式，或向双臂、升降（AtomW）下发位控运动命令。Host 订阅后调用位控算法并写 PLC 共享内存。

b. **话题名称**：`rt/posctl/cmd`

c. **IDL 文件内容**（`pos_control_cmd.idl`）：

```shell
module dobot_atom{
    module msg{
        module dds_{

            enum PosControlCommandType {
                SWITCH_MODE,   /* 切换 MIT <-> 位控。payload: target_mode */
                MOVE_L,        /* 笛卡尔直线。payload: left_pose/right_pose/hips_height + speed */
                MOVE_J,        /* 关节空间。payload: left_q/right_q/hips_q（前缀长度合并） */
                RUN_TO,        /* 笛卡尔跟踪。payload: 同上；stop=0 开始，stop=1 停止 */
                CART_JOG,      /* 笛卡尔点动。payload: left_cart_jog/right_cart_jog/hips_cart_jog；stop 同上 */
                STOP_MOTION,   /* 停止当前位控运动（可选扩展） */
                SERVO_J        /* 关节空间流式伺服。payload: left_q/right_q/hips_q（同 MOVE_J）；stop 同 RUN_TO；须 ~10Hz 保活 */
            };

            @extensibility(FINAL)
            struct PosControlCmd_ {
                PosControlCommandType command_type;

                /* ---------- MOVE_J / SERVO_J ---------- */
                sequence<double> left_q;    /* 1..7，前缀合并；0=左臂不参与；SERVO_J 复用 */
                sequence<double> right_q;   /* 1..7；SERVO_J 复用 */
                sequence<double> hips_q;    /* 1..3；SERVO_J 复用 */

                /* ---------- MOVE_L / RUN_TO ---------- */
                sequence<double> left_pose;   /* 0 或 6 */
                sequence<double> right_pose;  /* 0 或 6 */
                sequence<double> hips_height; /* 0 或 1，允许值为 0.0 mm */

                /* ---------- CART_JOG ---------- */
                sequence<int32> left_cart_jog;  /* 0 或 6，取值 -1/0/1 */
                sequence<int32> right_cart_jog;   /* 0 或 6 */
                sequence<int32> hips_cart_jog;    /* 0 或 1 */

                /** RUN_TO / CART_JOG / SERVO_J：0=开始，1=停止 */
                int32 stop;

                double speed;       /* MOVE_L / MOVE_J / RUN_TO 速度（算法解释，通常 0~1） */
                int32  target_mode; /* SWITCH_MODE：0=MIT，1=位控 */

                uint32 command_id;  
                uint64 timestamp;  
            };
        };
    };
};

```

i. 参考上述 IDL，目前包含七种命令：

1. `SWITCH_MODE`：MIT ↔ 位控切换（`target_mode`：0=MIT，1=位控）
2. `MOVE_L`：笛卡尔直线，单次下发
3. `MOVE_J`：关节点到点（单次规划），`left_q`/`right_q`/`hips_q` 为关节角 **deg**
4. `RUN_TO`：笛卡尔跟踪，`stop=0` 开始、`stop=1` 停止；**进行中须 ~10Hz 重复下发**
5. `CART_JOG`：笛卡尔点动，各轴 **1/0/1**，`stop` 语义同 `RUN_TO`；**进行中须 ~10Hz 重复下发**
6. `STOP_MOTION`：停止全部位控运动
7. `SERVO_J`：关节空间流式伺服（算法 `ServoJ`），载荷与 `MOVE_J` 相同（`left_q`/`right_q`/`hips_q`）；`stop=0` 下发目标、`stop=1` 停止；**进行中须 50Hz 重复下发**，Host 若 **300ms** 内未收到新命令则自动 `Stop()`

ii. 部件参与规则：`left_*` / `right_*` / `hips_*` **sequence 长度为 0 表示该部件不参与**；至少填一组需要运动的部件。

iii. 运动命令前置条件：

1. 先 `SWITCH_MODE` + `target_mode=1`，轮询 `rt/posctl/state` 直至 `control_mode == 1`（切换异步，可能数秒）
2. 相关轴已上使能（`rt/enable/motors`）
3. `error_code` 均为 0；若有错误，发 `rt/clear/errors` 清错后再运动

iv. 对于 `SERVO_J` / `RUN_TO` / `CART_JOG`（流式命令，须保活）：

1. `stop=0` 时开始/继续：须以约 **10Hz** **周期重复发布**同类型命令，并携带当前目标（关节角或笛卡尔位姿/点动方向）
2. Host 监测命令间隔：**300ms** 内无新的 `SERVO_J` / `RUN_TO` / `CART_JOG` 则自动停止运动（等效 `STOP_MOTION`）
3. `stop=1` 时停止：**发一次即可**（可不携带目标字段）
4. 也可用 `STOP_MOTION` 或任意命令 `stop!=0` 统一停止

### 示例 1：切换到位控应用

```plaintext
command_type  = SWITCH_MODE
target_mode   = 1          // 1=位控，0=MIT
command_id    = 1
```

发布后轮询 `rt/posctl/state`，直至 `control_mode == 1`。

### 示例 2：MOVE\_J（仅左臂 7 关节）

```plaintext
command_type  = MOVE_J
left_q        = [0, -30, 0, 90, 0, 0, 0]   // deg
speed         = 0.5
command_id    = 2
```

仅动左臂关节 0 时，可只填前缀：`left_q = [10]`（未给出的关节保持当前反馈角）。

### 示例 3：MOVE\_L（仅右臂笛卡尔直线）

```plaintext
command_type  = MOVE_L
right_pose    = [300, 200, 160, 0, 0, 0]   // [x,y,z] mm + [rx,ry,rz] deg
speed         = 0.5
command_id    = 3
```

### 示例 4：RUN\_TO 开始 / 停止（仅左臂跟踪）

开始：

```plaintext
command_type  = RUN_TO
left_pose     = [283.74, 206.5, 157.8, 0, 0, 0]
stop          = 0
command_id    = 4
```

停止：

```plaintext
command_type  = RUN_TO
stop          = 1
command_id    = 5
```

### 示例 5：CART\_JOG 点动（左臂 +X 方向）

开始：

```plaintext
command_type     = CART_JOG
left_cart_jog    = [1, 0, 0, 0, 0, 0]   // 各维 -1 / 0 / 1
stop             = 0
command_id       = 6
```

停止：

```plaintext
command_type  = CART_JOG
stop          = 1
command_id    = 7
```

### 示例 6：STOP\_MOTION（停止全部位控运动）

```plaintext
command_type  = STOP_MOTION
command_id    = 8
```

## 20 位控状态

a. **功能说明**：Host 周期发布位控应用模式、双臂/升降关节角、笛卡尔位姿、算法错误码与规划状态。

b. **话题名称**：`rt/posctl/state`

c. **IDL 文件内容**（`pos_control_state.idl`）：

```shell
module dobot_atom{
    module msg{
        module dds_{
            struct CartTargetPose {
                double left_pose[6];   // mm, deg
                double right_pose[6];
                double hips_height;    // mm
            };

            struct PosControlJointQ {
                double left_q[7];    // deg
                double right_q[7];
                double hips_q[3];
            };

            struct PosControlState_ {
                CartTargetPose pose;    //笛卡尔位姿
                int32 error_code[3];    // [0]左臂 [1]右臂 [2]升降
                int32 status_code[3];   // 同轴序
                int32 control_mode;     // 0=MIT，1=位控
                PosControlJointQ joint_q;//关节角度
            };
        };
    };
};
```

i. `control_mode`：`0` MIT 运控，`1` 位控应用

ii. `error_code` 取值（任一路非 0 应停止发运动命令）：

| 值 | 含义                |
| -- | ------------------- |
| 0  | 正常                |
| 1  | 初始化失败          |
| 2  | 伺服/使能错误       |
| 3  | 逆解失败            |
| 4  | 规划失败            |
| 5  | 参数维度/长度不合法 |
| 6  | 臂关节参考速度超限  |

iii. `status_code` 取值：

| 值 | 含义                               |
| -- | ---------------------------------- |
| 0  | Ready，可接受新命令                |
| 1  | Running                            |
| 2  | PlanStop（减速停止中）             |
| 3  | Error（结合 `error_code` 处理） |

iv. 如何判断模式切换完成？-

1. 订阅 `rt/posctl/state`，`control_mode` 变为 `1` 表示已进入位控应用

v. 如何判断单次运动结束？

1. 对应部件 `status_code[i]` 变为 `0`（Ready）表示该次运动结束
