# RM2022-Standard1
## 使用struct_typedef.h中的宏定义切换环步或张步的条件编译
## 介绍
GKD战队RoboMaster2022赛季，步兵机器人，C板（STM32F407IGHx），FreeRTOS，在官步基础上修改

## 软件和固件库版本
CubeMX 6.4.0  
STM32CubeF4 Firmware Package V1.26.0  

## 现有外设占用
#### 串口
1. USART3：遥控器
2. USART6(板上丝印UART1)：裁判系统
3. USART1(板上丝印UART2)：空闲
#### CAN
1. CAN1：chassis底盘(4个3508)
    - 将增加：超级电容
2. CAN2：视觉，gimbal云台(2个6020), trigger拨弹电机(1个2006)，fric摩擦轮电机(2个3508)  
(摩擦轮电机使用PWM控制的2个3508)  

## 开发记录
- 1.12 by 片哥
  - 在bsp_buzzer.c中禁用了蜂鸣器
  - 在INS_task.c中添加了变量Angle，用于保存以degree为单位的角度
- 1.13 by 片哥
  - 增加了结构体can_CV_t(最大64bytes)和gimbal_CV_t(大小不限)，并建立了数据流，将所有与CV相关的数据存在gimbal_task.c的结构体gimbal_control_t中的can_CV中 **Tips.CAN包最大64字节，现已用48字节**
  - 确定了视觉控制的逻辑：操作手通过键盘/遥控器开启或关闭自瞄，自瞄开启后云台和射击的控制由视觉接管，通过视觉算法控制姿态和射击
- 1.14 by 片哥
  - 官步使用一个微动开关BUTTON_TRIG_Pin检测送弹完成，因无需送弹，所以已修改此功能
    - 更改了shoot_bullet_control()中的SHOOT_DONE判断，原逻辑为微动开关松开即判断为完成一发子弹的射击，现逻辑为拨弹电机转动一定时间(TRIGGER_LONG_TIME)即判断为完成一次射击，不精确控制射出子弹量
    - 修改shoot_feedback_update，使shoot_control.key的状态一直处于SWITCH_TRIGGER_ON，屏蔽送弹检测
- 1.15 by 片哥
  - **右switch置下开启CV模式，只有在摩擦轮开启时才可由CV触发射击**，进入CV状态的判断写在gimbal_behavour_set中
  - 新增视觉控制射击的逻辑: 使用视觉回传的射击指令模拟switch down，从而触发射击
  - 修改了CV结构体中cv_reco_status的逻辑，不再由CV设备提供此信息，改为根据switch的值修改cv_reco_status，判断是否开启CV模式
  - 现有的CV模式只会控制云台，不会控制底盘运动，应在后续加入(已加入)，**思路是将CV模式视为直接控制遥控器左摇杆(即控制云台抬头低头、左转右转以及底盘跟随)，CV模式下操作手仅能控制右摇杆(与左摇杆互不冲突)**
- 1.18 by 片哥
  - 调试了视觉到电控的CAN通讯(开发板必须外接电池，暂不明确是否与CAN电阻有关)，在CAN回调中添加了接收的逻辑
  - 在user_lib中添加了kalman滤波，用于对接收到的视觉坐标进行滤波，在视觉端滤波算法
  - 调试了视觉的控制算法，尝试了原始值、位置式PID、增量式PID，最后选用了单环增量式PID，还需进一步优化，预计需要添加速度环
- 1.19 by 片哥
  - 修复了CV模式进入后无法退出的bug，并添加了CV数据接收超时判断
  - 添加了借助USART1的虚拟示波器支持(详见bsp_usart.c)
  - 调节了底盘的PID参数，云台和底盘现在使用CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW + GIMBAL_ABSOLUTE_ANGLE (switch置中)
- 3.3 by 片哥
  - 关闭了USB任务、OLED任务、电压检测任务
  - 增加CHASSIS_SPIN模式
  - **右switch上开启底盘小陀螺**
  - 修改gimbal_absolute_angle_limit()以适配小陀螺模式
  - 修改chassis_behaviour_mode_set()使小陀螺结束时只有完成一圈才会停止
- 3.6 by 片哥
  - 修改chassis_set_contorl()中CHASSIS_VECTOR_NO_FOLLOW_YAW的处理逻辑，使小陀螺的同时可以移动
  - 增加CAN2，启用FIFO1
- 3.7 by 片哥
  - motor_chassis结构体中增加摩擦轮
  - 增加CAN_cmd_fric()函数
  - 在gimbal_control_t结构体中增加摩擦轮控制内容
  - 部分沿用了原有摩擦轮控制逻辑，改写了shoot_control_loop()函数，用于速度控制的宏定义从bsp_fric.h移至shoot.h
  - 加入USB虚拟示波器调试功能
- 3.15 by 片哥
  - 增加用于优化键盘控制的斜波函数，更改宏定义RAMP_KEY_ADD_VX和RAMP_KEY_ADD_VY可以调整加速度
- 3.16 by 片哥
  - 继续优化键盘操作逻辑，加入缓停止
  - 将键盘控制设置为更高优先级
- 3.20 by 片哥
  - 云台CV模式改为使用absolute angle
  - 重构CV控制逻辑，在gimbal_control_loop()中增加gimbal_motor_cv_control()，删除gimbal_set_control()中的两处CV控制函数，CV控制在gimbal_control_loop()中完成
- 3.23 by 片哥
  - 缓解了yaw云台运动时的回正现象：修改底盘默认模式为CHASSIS_NO_FOLLOW_YAW，并将set_cali_gimbal_hook()中yaw motor的relative angle的默认值修改为一个较小值 (**这一修改会导致CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW模式运动缓慢**)
- 3.28 by 片哥
  - 修改了云台和底盘的跟随逻辑，不再强制限制角度，底盘CHASSIS_VECTOR_NO_FOLLOW_YAW模式中的角度改为由遥控器和角度跟随PID联合控制，基本消除了yaw云台回正现象
- 3.29 by 片哥
  - 添加了gimbal_yaw_direction，用于记录yaw云台向前或向后
  - 现在摇杆置中将会立即切换到CHASSIS_NO_FOLLOW_YAW模式
  - 小陀螺加入缓启停

## 笔记
### gimbal_task 云台控制任务
1. 云台的控制通过结构体**gimbal_control**完成，存放了遥控器/键鼠、陀螺仪、电机的信息
2. **gimbal_set_mode**() -> gimbal_behaviour_mode_set() -> 
   - gimbal_behavour_set(): 云台行为状态机，根据遥控器switch位置确定并通过变量gimbal_behaviour返回当前的云台行为(e.g., GIMBAL_RELATIVE_ANGLE / GIMBAL_ABSOLUTE_ANGLE)
   - 根据变量gimbal_behaviour的值设置每个云台电机的状态，存入结构体中的变量gimbal_motor_mode
3. **gimbal_mode_change_control_transit**() ->
   - 判断结构体gimbal_control中gimbal_motor_mode和last_gimbal_motor_mode是否一致，若不一致，则说明发生了模式切换，即进行数据保存和处理
4. **gimbal_feedback_update**() ->
   - 将CAN总线接收到的各个云台电机的数据进行解码，计算出相对角度、绝对角度、速度等信息，并存入结构体中的相应变量
5. **gimbal_set_control**() ->
   - 新建变量add_yaw_angle和add_pitch_angle，用于存放两个角度的增量值
   - gimbal_behaviour_control_set() ->
     - 根据**云台模式gimbal_behaviour**进入相应的gimbal_xxxxx_control()函数，在函数内使用不同方式根据遥控器/键鼠的操作得到云台pitch和yaw角度的增量，使用指针yaw和pitch返回
   - 根据**电机模式gimbal_motor_mode**决定控制方式，将变量add_yaw_angle和add_pitch_angle中的值存入结构体，用于下面的PID计算
   - 有两种模式参数：云台模式、电机模式
6. **gimbal_control_loop**() ->
   - 根据结构体中gimbal_motor_mode的值进入不同的gimbal_motor_xxxxx_angle_control()函数，根据上一步算出的角度值计算得到current_set/given_current控制值 **Tips.电视联控在这里添加**  
   - *(所有xxx_control_loop函数的逻辑都是相似的)*
7. **shoot_control_loop**() -> (详见程序注释)
   - shoot_set_mode()：状态机函数，用于判断并更新状态
   - shoot_feedback_update()：更新数据
   - 根据状态计算控制量
   - 控制量输出

### chassis_task 底盘控制任务
1. **chassis_task()**
2. **chassis_mode_change_control_transit()**
3. **chassis_feedback_update()**
4. **chassis_set_contorl()** ->
   - 新建三个局部变量，分别存放x轴速度，y轴速度，以及角度设定值
   - chassis_behaviour_control_set() ->
     - 根据**底盘模式chassis_behaviour_mode(单独定义在文件开头的变量)**进入相应的chassis_xxxxx_control()函数，在函数内使用不同方式根据遥控器/键鼠的操作得到vx,vy,angle，使用指针返回数值
     - 根据**电机模式chassis_move_control->chassis_mode**决定控制方式，将三个参数存入结构体，用于下面的PID计算
     - 有两种模式参数：底盘模式、电机模式
5. **chassis_control_loop()** ->
   - PID控制，除了RAW模式之外，其他模式的处理方法都是一样的

### 底盘和云台的协同控制
- relative_angle: 相对角度，来自6020编码器
- absolute_angle: 绝对角度，来自陀螺仪
1. 常用的云台模式:
   1. GIMBAL_RELATIVE_ANGLE: 云台使用**编码器**的相对角度控制旋转量
   2. GIMBAL_ABSOLUTE_ANGLE: 云台使用**陀螺仪**的绝对角度控制旋转量
   3. GIMBAL_CV: 视觉控制模式，坐标差转换为云台角度增量
2. 常用的底盘模式:
   1. CHASSIS_NO_FOLLOW_YAW: 底盘不跟随云台(没有角度闭环)，云台和底盘会根据遥控器摇杆值分别旋转(云台与底盘的角度限制已去掉)，底盘旋转靠双环控制(摇杆值+底盘回正PID)
   2. CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW：底盘通过**编码器**的relative_angle跟随云台(通过旋转底盘使relative_angle保持0)
- 环步使用的控制方式(2022.1.19):
  1. 右switch_up(**小陀螺**): CHASSIS_SPIN(基于CHASSIS_NO_FOLLOW_YAW) + GIMBAL_ABSOLUTE_ANGLE
  2. 右switch_mid(**普通模式**): CHASSIS_NO_FOLLOW_YAW + GIMBAL_ABSOLUTE_ANGLE
  3. 右switch_down(**CV控制**): CHASSIS_NO_FOLLOW_YAW + GIMBAL_CV(基于GIMBAL_ABSOLUTE_ANGLE)
  - *不能使用CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW + GIMBAL_RELATIVE_ANGLE！*

### 云台电机PID
每个轴都有三种PID，会同时使用1和3 或 2和3.
1. gimbal_motor_absolute_angle_pid
2. gimbal_motor_relative_angle_pid
3. gimbal_motor_gyro_pid

### CV控制逻辑
- 使用遥控器逻辑，NX发送摇杆值到32
- 右摇杆置下开启CV

### 操作手控制逻辑
- 平移：遥控器-右摇杆，键鼠-WASD
  - 对于平移，键鼠优先级高于遥控器，使用键鼠控制时会暂时禁用遥控器，一段时间未收到键鼠信号，则启用遥控器
- 视角：遥控器-左摇杆，键鼠-鼠标
  - 对于视角，遥控器和键鼠优先级相同，互不冲突

### 操作手
#### 自定义按键
- SHIFT: **预留**超级电容
- CTRL: **预留**小陀螺
- Q: 切换单发点射和连射模式(切换)
- E
- R
- F
- G:
- Z
- X
- C
- V
- B
#### 学生端按键
- I: 补充42mm弹丸
- K: 使用血包
- F12: 帮助界面

#### 自定义UI
- 摩擦轮实时转速(TODO)

## 参考资料
1. 官方2019步兵开源学习: https://bbs.robomaster.com/forum.php?mod=viewthread&tid=8361
2. C语言之如何输出uint32_t和uint64_t和16进制: https://blog.csdn.net/u011068702/article/details/77938756 
3. 【山东理工大学】自定义UI开源：https://bbs.robomaster.com/forum.php?mod=viewthread&tid=11924
4. Robomatser 画自定义UI界面：https://blog.csdn.net/dbqwcl/article/details/115518917
