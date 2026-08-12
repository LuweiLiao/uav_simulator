# Scorpio Gazebo 模型

本目录使用 `model.rsdf` 作为唯一人工维护的模型源文件。`model.sdf` 是 Gazebo
Classic 必需的运行时生成物，由构建任务从 RSDF 自动生成，不应直接修改。

## 模型结构

- `scorpio`：六足、三个倾转组件、三个旋翼及 SITL 接口的顶层装配。
- `scorpio_base`：新机身 DAE 与 IMU。
- `scorpio_coxa`、`scorpio_femur`、`scorpio_tibia`：单腿三个连杆。
- `scorpio_tilt`、`scorpio_prop`：单个倾转机构和旋翼。

## 通道顺序

- 腿：`0 RF、1 RB、2 LB、3 LF、4 RM、5 LM`，与 `AP_HexRuped` 一致。
- 旋翼/倾转：从机体右侧开始俯视顺时针，`0 前右、1 后中、2 前左`。

## 实物标定入口

六足安装点、腿段关节中心间距、关节限位、三旋翼安装点、倾转轴、旋翼方向和
动力参数均集中在 `models/scorpio/model.rsdf` 顶部。机身质量、碰撞盒、惯量、
DAE 缩放和坐标补偿集中在 `models/scorpio_base/model.rsdf` 顶部。

六个 Coxa 根部共用 `leg_mount_z`，当前为 `0.000 m`；Coxa 轴由
`coxa_axis_x/y/z` 定义，默认 `(0, 0, -1)`，与 body Z 轴平行。

机身 DAE 已保留 CAD 原点，`body_mesh_x/y/z` 必须保持为 0；DAE 带有 `Y_UP`
元数据，Gazebo 导入时会处理它，`body_mesh_roll` 也必须保持为 0。这样 Gazebo
的 `scorpio_base::base` 原点与实物设计坐标原点重合。

当前腿根坐标由新机身 DAE 的安装圆心测得；三处倾转铰点采用 CAD 实测值，
坐标按 `Gazebo (X,Y,Z) = CAD (Z,X,Y)` 转换：前部为
`(0.18811, ±0.20529, 0.14866) m`；前右安装角为 `+60°`、前左为
`-60°`。后部为
`(-0.22917, 0, 0.13866) m`。腿段长度仍沿用原模型，
拿到实物测量值后只需修改上述参数区，不需要改装配 XML。

倾转 DAE 的局部 Y 轴为铰轴，顶层装配的 yaw 按机臂方位设置，使铰轴与机臂
切向平行并穿过白色端部支架的安装孔。

## 构建与启动

```bash
cd /home/pix/uavros_ws
task erb-Scorpio:Scorpio
roslaunch uav_gazebo spawn.launch world_name:=Scorpio
```

每次修改 RSDF 后重新执行第一条命令。启动命令会加载
`uav_gazebo/worlds/Scorpio.world`，并通过 Scorpio 专用模型搜索路径找到全部子模型。

## RSDF 展开检查

可以将展开结果写到临时目录进行语法检查：

```bash
erb models/scorpio/model.rsdf > /tmp/scorpio.sdf
xmllint --noout /tmp/scorpio.sdf
```

顶层使用独立的 `libArduRotorScorpio.so` 作为 SITL 传输插件，不修改原四足双旋翼
插件。模型中已经声明 6+6+6 个腿关节、3 个旋翼和 3 个倾转关节。Scorpio 打开 SocketCAN 模式后，
标准 Gazebo UDP 的 16 路 PWM 中第 1–3 路控制旋翼、第 4–6 路控制倾转；18 个腿关节
通过 `vcan0` 上的 DroneCAN `com.usl.ServoCmd` 传输。插件按模型配置动态发布六路
Coxa/Femur/Tibia 命令，关闭 SocketCAN 参数时仍兼容原四足 UDP 布局。
