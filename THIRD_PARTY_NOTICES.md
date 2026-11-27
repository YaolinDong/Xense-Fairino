# 第三方资源与许可

- `src/lerobot/` 许可见 [LICENSE](vendor/lerobot_source/LICENSE)。
- `vendor/fairino/` 为法奥数学库及头文件，遵守原厂许可。
- `umi/vendor/TacCap-Gripper/` 为 XenseRobotics-AI/TacCap-Gripper 0.4.3 源码快照，revision `49d6a6ba587f13c97e145725034360a44d590013`，保留上游 [Apache-2.0 许可](umi/vendor/TacCap-Gripper/LICENSE)。
- PICO SDK 与绑定位于 `src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/`，保留源文件许可。
- PC Service/APK 版本、URL 与校验见 [assets.json](scripts/pico4/assets.json)，安装到项目 `.local/pico4/`。
- ADB 来自 Google 官方 Android Platform Tools。
- 雷达图来源见 [sources.json](docs/images/sources.json)，模型校验见 [manifest.json](models/manifest.json)。
- `third_party/fairino_ik/` 是 git submodule，指向内部仓库 `XenseRobotics-AI/fairino_ik` 的 `main` 分支，固定在提交 `dd8cdc5d69e74f19514ff63077c69b752f5f316c`。该提交未附 LICENSE 文件。

第三方资源原有许可不因项目集成而改变。
