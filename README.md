# 法奥机器人控制项目

一个仓库包含底盘键控、20 轴单关节控制、身体高度与姿态联动、双臂 TCP 键控、PICO 双臂遥操、双雷达显示、按需建图、AMCL 定位、二维导航、故障清除和链路诊断。安装、启动、配置、模型及离线验证均在本项目内完成。

## 集成工作台

```bash
bash setup.sh console             # 首次安装桌面绘图依赖
bash run.sh console               # 打开时不连接、不使能
bash run.sh console --demo        # 不连接机器人的界面演示
bash run.sh teleop --demo         # PICO 手柄控制台模型演示
GTK_THEME=Adwaita bash run.sh console  # Gruvbox 显示异常时，用 Adwaita 启动工作台
```

PICO 页新增「运行目标 → 控制台模型（Demo）」。启动 PICO 接收服务后，头显开启 Head / Controller / Send，点击「启动模型 Demo」即可控制右侧模型；无需机器人或 SSH 密码。默认面对面映射、灵敏度 0.25，双臂浅绿色。Demo 可以关闭腕部联合限位：模型允许进入联合限位定义的干涉区，相应腕心变红；退出干涉区恢复绿色。红色依据腕部限位模型，并非整机网格碰撞检测。使用说明见 [SLAM 与 PICO 模型演示](docs/SLAM与PICO模型演示.md)。

真机和 Demo 启动前均可选择「联合限位 IK」「原厂 IK」「TRON2 QP IK」或「Fairino QP IK」，运行中锁定，结束后可切换，默认仍为联合限位 IK。TRON2 使用原包 C++ QP 核心和当前 ART7 模型；首次安装见 [接入说明](docs/SLAM与PICO模型演示.md#tron2-qp-后端)。TRON2 真机强制开启外层腕部路径保护；Demo 可开关，关闭不替换算法。下方状态显示实际算法及保护状态。

顶部填写 SSH 密码后点击“连接 / 刷新只读状态”，即可在一个窗口查看地图、双雷达、两块控制板的使能/故障状态和连接类型（有线/无线、网卡、本机 IP、机器人 IP）。密码只保存在进程内存，不写入配置或日志。

“建图与底盘”页同时显示地图、双雷达和底盘键控：上方查看地图，下方启动/结束、调速和按键移动，不需要切页。原来的 `--page base` 也进入此页。关节、身体、TCP、遥操各页保留显式的启动/结束按钮。上半身同一时间由一个控制模块占用；必须先结束当前模块并等待退出，再启动另一个。PICO 页可启动接收服务、启动遥操、暂停、请求 Home 和调灵敏度，默认勾选“先回 Home”，该动作会运动。高频遥操仍由现有 `vr_client.py` 独立进程执行。

底盘、关节、身体和 TCP 面板打开工作台时就完整显示；未启动时控件置灰，只能查看。点击上方启动按钮并等待控制器就绪后开放操作，启动失败、结束控制后恢复置灰显示。预览复用独立程序的界面，不建立机器人控制连接、不使能，也不运行模拟反馈。

键控切换页面或窗口时暂停运动、保持使能，切回后松键重按；身体联动需重新点击执行。退出控制、关闭工作台、断线或故障仍走各模块的结束与下使能流程。PICO 使用手柄 Grip 和暂停按钮控制跟随。

整机视图使用与已校验 URDF 完全匹配的 31 个厂家 STL 外观网格，在电脑端用 OpenGL 渲染；机器人只提供只读状态，无需安装渲染程序。支持左键旋转、右键平移、滚轮缩放、视角复位和双臂分组着色。工作台使用 RD53 20261030 整机模型，J1–J14 为双臂、J15 为头部、J16 为腰旋转、J17 为侧倾、J18–J20 为折叠关节。双臂模型与现有厂家正解已离线核对；折叠关节复用身体程序的 `body_config.json` 及竖直参考，同时显示高度百分比、相对低位升高量和姿态。毫米数为模型估算，不是实测离地高度。轮组为静态参考。反馈过期时模型变灰并保留最后姿态。`--static-model` 可关闭实时模型显示。

独立 `base / joints / body / tcp / map / vr / errors` 入口保留。工作台嵌入原有控制界面，地图窗口使用同一个 `MapPanel`，清错调用同一份远端实现，因此修改功能模块会同时影响集成与独立运行。

关节、TCP、PICO 共用可切换的腕部联合限位模块，默认开启。
在界面运行中启用 / 停用会先暂停、保持使能，再确认双端配置；重新输入才继续。
它检查 J6/J7、J13/J14 的组合及经过路径，PICO 限制后不补走旧手柄位移，
Home 先检查整条路径。范围、模型余量与验证限制见 [腕部保护](docs/腕部保护.md)。

新增「夹爪调试」页，也可单独运行 `bash run.sh gripper`：按固件 SN 识别左右 TacCap / XTac，从只读连接开始，分别使能、调节连续开度、闭合、保持和下使能。与 PICO 共用同一 SDK 接口。

整机视图已加入用户 STEP 提供的夹爪和转接件，随手臂运动；夹指按实测开度更新，
内部连杆使用近似几何。工作台「夹爪调试」只读连接或 PICO 夹爪会话提供反馈，
显示复用同一个会话，不重复占用串口；超过 0.5 秒没有有效反馈时保留最后开度并变灰。
PICO 页支持选择固定指尖中心 TCP、分别调整左右安装角度、最大开度百分比，以及夹爪速度
（默认 0.75 rad/s，范围 0.01–1.0 rad/s）。最大开度分别限制左右 Trigger 的映射上限，
100% 对应各夹爪的标定全开；界面反馈同时显示对应的电机转角。安装角度和固定工具点需核对后再用于实机，
显示与控制共用配置。详见 [夹爪模型与 TCP](models/taccap/README.md)。

PICO 页的「双臂起始位姿」保留原 Home 为默认，并可选「起始位姿 2 · 10-20 右臂实测镜像」。
第二项来自 2026-10-20 14:18 的右臂静止实测，左臂按躯干中线镜像；只改变 J1–J14。
自动回 Home、界面回 Home 和右摇杆回 Home 均使用启动前所选项。第二项已做厂家 FK/IK、
关节限位及镜像校验，未执行实机轨迹，需自行确认手臂与夹爪路径的空间。

PICO 页显式勾选夹爪和身体升降后再启动：食指 Trigger 线性控制对应夹爪（松开张开、压到底闭合），Grip 继续控制手臂跟随；右摇杆上下控制标定范围内的竖直升降，回中保持。身体升降复用身体程序的模型和标定，与双臂共用一个二十轴控制实例。命令行对应 `bash run.sh vr --grippers --body-lift`，选项默认关闭，开窗口不会使能。详见 [PICO 说明](scripts/pico4/README.md) 与 [夹爪说明](apps/gripper_control/README.md)。

## 统一底盘与里程计配置

新底盘程序只使用 `eight_dof_chassis_controller`，原始里程计固定为 `/eight_dof_chassis_controller/odom`。第一次使用前需要迁移机器人配置；默认命令仅预览，`--execute` 才写入，均不重启驱动、不使能：

```bash
.venv/bin/python tools/install_control_stack.py
.venv/bin/python tools/install_control_stack.py --execute
```

已完成底盘配置迁移后，只切换里程计可加 `--apply-now`：例如 `.venv/bin/python tools/install_control_stack.py --execute --odometry raw --apply-now`。工具先确认控制空闲、静止、下使能且未建图，再仅重启里程计；临时 `fairino-odometry-live.service` 随 `amr_startup` 停止，下次开机仍由正常启动脚本读取 `mode.json`。其他底盘启动配置不会因此热更新。

安装器备份原文件，关闭底盘开机自动执行的使能序列，并将里程计交给 `odom_guard.py` 管理。默认 `--odometry raw` 使用原始轮速；`--odometry ekf` 保留原融合方案用于后续调试，两种模式互斥。配置在里程计守护程序重新启动后生效；安装器本身不会重启硬件。通常在静止、全部下使能后重启机器人应用。不要通过重启旧的 `amr_startup` 来试错，旧配置可能自动使能。安装回执给出备份目录与文件清单。

键控显式启动后才回正/使能；复用厂家底盘实例；正常退出确认下使能且反馈健康后保留其里程计测量，异常或启动失败时仍停用，不卸载该实例。清错先确认运动控制已停止、机器人静止，再单次复位；故障归零后按需下使能，最后复查，结束保持停用。允许部分轴仍使能时进入清错，避免故障导致厂家 `disable()` 返回 `0xC012` 而无法复位。复位失败、故障复发或下使能失败均停止，不自动重复清错。

里程计在轮速和硬件反馈连续健康一秒后开始输出。原始模式保留厂家 odom 的 x/y、朝向、速度、协方差及原始时间戳，通过 `/odom` 和 `odom → base_footprint` TF 提供给建图；只核验静态坐标关系并投影到地面，不做积分、融合或旧速度外推。断流、坏时间戳或硬件故障时停止，恢复数据不会自动继续。下一次显式启动底盘会请求新里程计会话。建图检测到会话变化后拒绝“继续建图”，必须先另存/结束旧工作地图，再准备新建图。

建图步骤：连接只读状态 → 清错并复查 → “建图与底盘”页启动底盘 → 确认原始 odom 和“建图 odom（原始直通）”更新 → “准备新建图” → 在同页按键低速移动、查看地图 → “暂停建图” → “另存地图” → “结束建图 / 返回底图”。默认不建图；冻结显示只冻结画面。暂停时顶部用红色显示“不再采集”。保存不覆盖默认地图，关闭查看窗口不停止其他程序启动的控制。

2026-10-18 起默认采用“厂家原始轮速 + 雷达 SLAM”，不依赖 EKF，也不应用 EKF 的 `1 m²` 暂停阈值。保留轮速、直通输出、雷达的新鲜度/时间戳检查、TF 检查、硬件故障与会话保护。地图实际精度仍需直线、转向和回到起点验证。原 EKF 问题分析见 [故障排查](docs/故障排查.md#建图很快因位置不确定度过大而暂停)。

建图采用独立的 SLAM 扫描筛选修复，支持原地转向选帧，约 10 厘米 / 5° 取新帧、每秒发布地图。界面显示成功雷达匹配时间，运动后没有匹配时明确提示 TF 随轮速推算；静止时正常等待下一关键帧。部署与参数见 [地图程序说明](apps/map_viewer/README.md)。

2026-10-22 已将另一台控制电脑上的地图、定位和导航改动合并到本地。10 月 19 日的历史部署记录确认修正了两个雷达相对车身坐标的 90° 方向差；本地同步了对应 URDF。该记录证明当时软件 TF 已应用指定值，不代表此前约 88° 的轨迹偏差已经通过运动测试消除。新测试应重新建图，不继续使用旧外参下的地图；参数及证据见 [雷达与模型](docs/雷达与模型.md)。

已有地图定位使用 AMCL 全向模型，支持地图选择、启停、初始位姿输入/拖拽和状态检查。默认地图可以为空；新地图建好并明确选用后再启动定位。二维导航已接入 `slam` 分支：优先直线、受阻时 Smac2D 绕行，使用全向预瞄控制器（3 Hz），默认限速 0.10 m/s / 0.20 rad/s。当前只检查底盘二维轮廓，不包含伸出的双臂。工作台保留图层、目标红叉和路径预览，并增加保存位置点；配置和部署源码统一位于 `modules/fairino_slam/`，入口 `bash run.sh slam help`。

本次仅同步本地代码。远端保存了此前定位/导航的部署与服务检查记录，但最新导航连续执行参数仍待机器人端更新，实际定位、行驶和避障精度尚未完成验证。先停止控制及地图/定位/导航会话，再按 [部署步骤](apps/navigation/README.md#代码入口与部署) 更新机器人端程序。使用顺序为：选用新地图 → ① 接入导航底盘（回正使能）→ 启动定位并设置初始位姿 → 核对雷达与地图重合 → ② 启动导航 → 输入目标、预览路径 → ③ 开始导航。仅输入目标和预览路径不会移动；取消目标保持使能，结束导航请求下使能。完整操作见 [二维导航说明](apps/navigation/README.md#工作台操作)。

## 安装

支持 Ubuntu 24.04 x86_64、Python 3.12。首次安装需要联网。

```bash
git clone git@github.com:limfxin/fairino-robot-toolkit.git
cd fairino-robot-toolkit
sudo apt install python3-venv python3-dev python3-gi gir1.2-gtk-3.0 \
  python3-tk openssh-client curl build-essential cmake unzip libusb-1.0-0 fonts-noto-cjk
bash setup.sh all
bash run.sh test
```

`setup.sh all` 准备基础界面、离线诊断、PICO 接收服务、头显 APK、ADB 和 SDK 绑定。环境与缓存位于 `.venv/` 和 `.local/`，厂家运动学库及诊断模型随代码提供。

仅准备工作台可运行 `bash setup.sh console`，其中包含地图绘图需要的 GTK/Cairo 组件。工作目录通过符号链接复用已有 `.venv` 时，也会自动查找该环境所属目录的 `.local/gtk`。

## 功能与项目位置

以下命令均从仓库根目录执行，路径均相对于仓库根目录。

| 功能 | 项目内位置 | 入口 |
|---|---|---|
| 底盘 WASD/JL | [apps/fairino_keyboard/](apps/fairino_keyboard/) | `bash run.sh base` |
| J1–J20 单关节控制 | [apps/fairino_upper_probe/](apps/fairino_upper_probe/) | `bash run.sh joints --group body` |
| 身体高度、前倾、侧倾、腰旋转 | [apps/fairino_upper_probe/body_control/](apps/fairino_upper_probe/body_control/) | `bash run.sh body` |
| 双臂 TCP 十二键控制 | [apps/fairino_tcp_probe/](apps/fairino_tcp_probe/) | `bash run.sh tcp --arm left` |
| PICO 安装、接收与预览 | [scripts/pico4/](scripts/pico4/) | `bash run.sh pico service` |
| 双臂遥操、插值、Home、冻结 | [scripts/pico4/fairino/](scripts/pico4/fairino/) | `bash run.sh vr` |
| PICO 设备接入与双手柄 | [src/lerobot/teleoperators/](src/lerobot/teleoperators/) | 由 PICO 入口加载 |
| TacCap / XTac 夹爪调试 | [apps/gripper_control/](apps/gripper_control/) | 工作台「夹爪调试」 / `bash run.sh gripper` |
| 上半身和底盘清错 | [clear_errors.py](scripts/pico4/fairino/clear_errors.py) | `bash run.sh errors --execute` |
| 双雷达扫描显示 | [apps/lidar_viewer/](apps/lidar_viewer/) | `bash run.sh lidar` |
| 固定地图、实时雷达、建图与定位 | [apps/map_viewer/](apps/map_viewer/) | `bash run.sh map` / `mapping status` |
| 二维底盘导航 | [apps/navigation/](apps/navigation/) | `bash run.sh navigation status` / `plan` / `goal` |
| 网络与机器人状态 | [tools/](tools/) | `bash run.sh network` / `status` |
| 雷达与手腕离线诊断 | [diagnostics/](diagnostics/) | `bash run.sh diagnostics --help` |
| 上半身夹爪编译、扫描与捏合 | [umi/README.md](umi/README.md) | SSH 到上半身，按文档运行 |
| 模型、网格和样例 | [models/](models/)、[data/](data/) | 各模块自动按项目路径读取 |
| 全部离线验证 | [tests/](tests/) | `bash run.sh test` |

`base / joints / body / tcp` 普通启动会使能；`vr` 会使能并先回双臂 Home。只读检查用对应 `--check`，离线界面用 `--demo`。全部命令见 `bash run.sh help`。

## 开机后的故障检查

开机或重启后可能仍有驱动器故障。已记录过全局 `0x0E10FF01`、关节 `J15=30`；`30` 为各轴十进制错误码，现有故障表将其归为 EtherCAT 通信类故障。具体轴与原因以当次反馈为准。

```bash
bash run.sh errors
# 有故障时，先退出控制程序，确认静止、急停释放；允许部分轴仍使能
bash run.sh errors --execute
bash run.sh errors
```

清错最多请求一次复位，再按需下使能。只有复查全局和各轴错误码归零、机器人静止且全部下使能才报告完成。实测速度静止阈值为 0.05 rad/s，速度指令阈值仍为 0.005 rad/s；超出时显示具体关节与数值，故障复发时保留日志排查。工作台和独立清错命令共用这些阈值。

## PICO 双臂遥操

```bash
bash run.sh pico service  # 第一个终端保持运行
bash run.sh pico monitor # 另一个终端先确认数据，随后退出 monitor
bash run.sh vr
```

头显填写电脑可达的局域网地址，开启 Head、Controller Tracking 和 Send。「遥操方式」默认面对面镜像，右手柄控制左臂，左手柄控制右臂；可选第一人称（或 `bash run.sh vr --mode first_person`），同侧手柄控制同侧手臂和夹爪。Grip 握住跟随、松开保持。A/X 按住冻结旋转，B/Y 按住冻结位移；左手 X 双击降低灵敏度、Y 双击提高；左摇杆按压校准双臂遥操正前方，右摇杆按压回 Home。暂停使用空格或工作台「暂停 / 保持」。勾选「同时启动底盘」或加 `--chassis` 后，左摇杆全向平移、右摇杆左右旋转，回中停车；面对面模式的平移前后、左右相对第一人称均反向，旋转保持同向。方向不随头显转头改变，右摇杆上下仍用于可选的身体升降。运行中需先结束遥操再切换模式。

校准时松开两侧 Grip、两根摇杆回中并等待机器人停稳，平视希望的双臂遥操正前方，再按下物理左手柄摇杆。启用底盘时还会检查底盘已停止。校准本身不产生运动；释放摇杆按键后重新握住 Grip 才恢复双臂跟随。校准失败仍保持暂停，界面显示原因并保留上次有效校准。方向只在当前会话内固定，转头不会改变，退出遥操后失效；它不改变镜像／第一人称手臂路由，也不改变按机器人坐标定义的底盘摇杆映射。详见 [正前方校准](docs/PICO遥操有线方案与延迟说明.md#正前方校准)。

夹爪实际安装绑定为机器人左爪 `TCGU01A28Z0018s`、右爪 `TCGU01A28Z0015s`；调试页、遥操及模型反馈共用绑定，填写 SN 可覆盖。

`vr` 指定 450 Hz TCP 插值、IK 和下发，500 Hz 接收轮询，100 Hz 状态回传，12 ms 输入缓冲。链路为 TCP 位移线性插值／姿态 SLERP → 厂家 IK → 有线 SSH → 机器人 ROS 控制器 → EtherCAT。实际频率以日志为准。

## 说明与配置

- [安装与运行](docs/安装与运行.md)
- [功能汇总与模块索引](docs/功能汇总.md)
- [网络配置](docs/机器人网络配置与连接说明.md)
- [遥操与延迟](docs/PICO遥操有线方案与延迟说明.md)
- [雷达与模型](docs/雷达与模型.md)
- [故障排查](docs/故障排查.md)

双臂原 Home 在 [home_pose.json](scripts/pico4/fairino/home_pose.json)，第二项在 [home_pose_right_mirrored.json](scripts/pico4/fairino/home_pose_right_mirrored.json)；身体参数在 [body_config.json](apps/fairino_upper_probe/body_control/body_config.json)，依赖清单在 [requirements/](requirements/)。第三方许可见 [资源说明](THIRD_PARTY_NOTICES.md)。
