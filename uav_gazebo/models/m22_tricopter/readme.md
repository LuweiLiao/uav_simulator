# m22_tricopter

mk22 倾转三旋翼。网格由 Fusion 模型 `mk22 v13` 导出，外购件已简化为凸包，桨去掉了外缘圈。

坐标：X 机头，Y 左，Z 上，原点在下底板底面中心。

## 舵机编号（`/gazebo/command/tilt1_pos`）

| 编号 | 关节 | 转轴 |
|---|---|---|
| 0 | 右臂 pitch | -Y（Gazebo 系），即机体 +Y 为正，左右同号 |
| 1 | 右臂 roll（挂在右 pitch 件上） | +X（机体 +X 为正，左右同号） |
| 2 | 左臂 pitch | -Y（Gazebo 系），即机体 +Y 为正，左右同号 |
| 3 | 左臂 roll（挂在左 pitch 件上） | +X（机体 +X 为正，左右同号） |

约定：前右下机体系，pitch 绕机体 +Y 为正，roll 绕机体 +X 为正，左右同号。roll 限位暂定 ±90°。

## 电机编号（`/gazebo/command/prop_speed`）

| 编号 | 位置 | turningDirection |
|---|---|---|
| 0 | 右臂上桨 | ccw |
| 1 | 右臂下桨 | cw |
| 2 | 左臂上桨 | cw |
| 3 | 左臂下桨 | ccw |
| 4 | 后桨（可正反转） | ccw |

后桨可正反转：飞控插件里 `<reversible_motors>4</reversible_motors>`，把 4 号电机的飞控输出
0..1 映射成 -1..1 倍最大转速，0.5（PWM 1500）停转，小于 0.5 反转、推力朝下。
ArduPilot 端对应输出通道要按可逆电机（3D 电调）配置。直接发 ROS 话题时给负转速即可反转。

CAD 里同侧上下两副桨的桨叶旋向相同（右侧都是 ccw，左侧都是 cw），和共轴反转不符，
这里按共轴反转设置；实机装桨确定后改 `model.rsdf` 里的 `:dir`。

## 待实测的参数

- 各部件质量、质心、惯量：写在各子模型 `model.rsdf` 顶部，目前是估算值（机体 2.5 kg、pitch 件 0.3 kg、roll 件 0.45 kg、前桨 0.035 kg、后桨 0.01 kg）。
- 电机参数 `prop_motorConstant` 等沿用 tilt_tricopter，前后桨尺寸不同，需要按实测推力改。

## 生成 sdf

```bash
task erb-m22_tricopter:m22_tricopter
```

子模型和网格由 mk22 仓库的 `tools/gen_gazebo_submodels.py` 生成；重新运行会覆盖子模型的 `model.rsdf`，
手改过的质量参数要同步改到脚本里。总模型 `m22_tricopter/model.rsdf` 是手写的，脚本不会动它。

## 飞控插件

使用独立插件 `libArduRotorM22Tricopter.so`（`uav_gazebo_plugin/src/ArduRotorM22Tricopter.cc`），
由 `ArduRotorTiltTricopter` 复制而来，不和 tilt_tricopter 共用。相对原插件增加了：
`<reversible_motors>` 可正反转电机、按 `motor_num`/`servo_num` 实际数量发布话题、
没有前向测距时只告警、固件尚未输出（PWM 0）时电机停转舵机回中位。
