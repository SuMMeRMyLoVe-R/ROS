# s1110834_description

本專案為「ME382A 機器人作業系統」期末專題，開發一套具備避障、自動行走與手臂控制功能的 ROS 機器人模擬系統，並可於 Gazebo 環境中運行與展示。


本專案包含以下三項主要功能模組：

1. obstacle_move.py
   - 自動向前移動，並依據雷達資料進行避障。

2. arm_45degree.py
   - 控制機器手臂持續上下擺動，模擬服務或迎賓姿態。

3. obstacle.py 結合鍵盤控制）  
   - 使用 teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_raw 進行方向控制，並整合避障功能。

---

# 系統需求

- ROS Noetic
- Ubuntu 20.04
- Gazebo 11
- Python 3

---

# 套件安裝與建置

cd ~/catkin_ws/src
git clone https://github.com/SuMMeRMyLoVe-R/ROS.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash