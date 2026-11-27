# 运行与诊断模型

| 文件 | 用途 |
|---|---|
| [left_arm.urdf](left_arm.urdf)、[right_arm.urdf](right_arm.urdf) | 双臂独立模型验证 |
| [whole_body_20261030.urdf](whole_body_20261030.urdf) | 当前 RD53 整机模型，用于胸部安装、头腰映射和折叠机构 |
| [whole_body_20260731.urdf](whole_body_20260731.urdf) | 历史底盘分析和腕部保护标定来源 |
| [base_link.STL](meshes/base_link.STL) | 底盘/雷达机械位置分析 |
| [wrist1_Link.STL](meshes/wrist1_Link.STL)、[wrist2_Link.STL](meshes/wrist2_Link.STL) | 腕心和法兰偏移绘图 |
| [manifest.json](manifest.json) | 大小与 SHA-256 |
| [whole_body_meshes/](whole_body_meshes/) | 实体模型的 31 个厂家 STL；仅用于电脑端渲染 |

URDF 保持原始字节以支持哈希校验。全身控制编号、物理限位与动力学仍须结合实际设备确认。见 [雷达与模型](../docs/雷达与模型.md)。

整机外观来自 `RD53-轮足机器人URDF和模型-v1.0-20261030/fairino_wheel_legged_robot_description/`。该目录的 URDF 与项目运行 URDF 哈希完全一致，网格按原始字节复制；来源、大小及校验值见 [whole_body_meshes/manifest.json](whole_body_meshes/manifest.json)。运行时只读取项目内资源，不依赖 Documents 目录。外观变换使用 URDF 的 visual origin 和 mesh scale，关节变换复用工作台已有的实测映射；外观文件不参与运动控制。
