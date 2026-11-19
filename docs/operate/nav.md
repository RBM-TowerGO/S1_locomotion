网页导航
在 S1B 上启动：

￼
/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation.sh --mode hardware
需要网页时，在 S1B 的另一个终端运行

/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-navigation-web.sh --port 8767

在你的电脑上另开一个终端建隧道，然后浏览器打开 http://127.0.0.1:8767 ：

￼
ssh -N -L 8767:127.0.0.1:8767 S1B
想先只观察 scan 和定位、不加载 SDK，用 --mode observe。这次改了扫描来源，建议先这样静止看几分钟。
只检查配置和依赖、不启动任何节点，用 --check。
页面操作顺序：刷新页面（Ctrl+F5），在地图上标出机器人的实际位置和朝向，等定位就绪，设置速度（默认 0.05 m/s），选目标，点开始。
网页上按 Esc 或空格停止。在网页终端按 Ctrl+C 只会关闭网页；在导航终端按 Ctrl+C 结束导航会话。
键盘遥控
￼
ssh -t S1B '/home/astribot/TowerGO/S1_locomotion/workspace/src/astribot_navigation/scripts/tg-teleop.sh'
必须加 -t，脚本需要交互终端。不加载 SDK、只演练按键可以加 --dry-run。指定初始速度可以加 --linear-speed 0.03 --angular-speed 0.075。