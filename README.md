# S1_locomotion

TowerGO 的 S1 底盘建图、定位、导航与 VLM 导航工作区。

本仓库于 2026-11-15 从统一 TowerGO 仓库的 `S1_locomotion/` 目录独立导出。它包含 S1B 当前源码快照、配置、测试、文档和导航地图，各目录均为普通 Git 文件，不依赖子模块。

## 目录与入口

| 路径 | 内容 |
| --- | --- |
| `workspace/src/astribot_navigation/` | ROS 2 导航软件包、脚本和测试 |
| `workspace/data/maps/` | 现场导航地图及编辑记录 |
| `runtime_ws/` | 真机运行工作区布局及已有 Python 构建依赖 |
| `vlm_navigation/` | VLM 部署脚本、配置和已有检查报告 |
| `docs/operate/` | 现场操作说明 |
| `HANDOFF.md` | 历史现场调试与交接记录 |

- 导航操作：[docs/operate/nav.md](docs/operate/nav.md)。
- 工作区说明：[workspace/README.md](workspace/README.md)。
- 离线实验：[workspace/src/astribot_navigation/README.md](workspace/src/astribot_navigation/README.md)。
- VLM 部署：[vlm_navigation/README.md](vlm_navigation/README.md)。

子目录中保留了不同时期的文档；离线实验说明不覆盖后续增加的真机导航功能，历史交接中的运行状态也不代表当前机器人状态。

## 环境

目标环境为 Linux ARM64、ROS 2 Humble。原 S1B 部署路径是 `/home/astribot/TowerGO/S1_locomotion`，部分脚本保留机器绝对路径。独立克隆后需要调整路径、重建 ROS 工作区，并按相应模块说明配置外部环境。

Conda 环境、Astribot SDK、模型权重和系统依赖不随本仓库分发。完整 TowerGO 环境说明与顶层激活脚本位于原统一仓库，未将其复制成新的独立部署入口。

## 快照来源与验证

- 统一仓库：`RBM-TowerGO/TowerGO`，提交 `6a2c56ff1521748f5ab2de4d2f38e7d8c816d75b`，目录 `S1_locomotion/`。
- 原开发仓库：`RBM-TowerGO/Astribot_S1`，分支 `feat/vlm-chassis-navigation`，基准提交 `ef5707ec6c05082bd7f13e5837ad0ed93ca57c10`。
- 包含快照采集时尚未提交的导航源码改动，不导入旧仓库 Git 历史。
- 拆分时 S1B SSH 连接关闭，使用此前已成功复制并上传的快照，没有宣称重新获取到更晚的机器人文件。
- 2026-11-15 采集快照时，S1B 上 `tests/test_static_map_navigation.py` 的 8 项测试通过；使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 避开已有 ROS 插件与 pytest 的接口不兼容。本次仅拆分发布，未重复运行真机或全量测试。

`SOURCE_MANIFEST.json` 记录导入文件的 SHA-256。除本说明、清单和根 `.gitignore` 的补充排除规则外，源快照的文件内容保持不变。

## 未上传的内容

大型采集数据、`workspace/data/{cache,mapping,reports}/`、ROS 编译产物、运行日志、Python 缓存、凭据、原 `.git` 历史和机器本地备份继续保留在 S1B。导航地图以及 Docker 构建需要的固定版本 SLAM Toolbox 源码归档已包含。

这是源码快照，不是完整实验数据备份；历史文档中指向未上传数据的链接无法仅凭本仓库恢复。

各模块和第三方依赖保留其原有许可证，本次导出未新增覆盖全部内容的许可证。
