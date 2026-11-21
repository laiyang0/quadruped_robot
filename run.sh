colcon build --packages-select reference_trajectory_generator keyboard_input
colcon build --packages-select quadruped_robot_controller go2_description

ros2 launch quadruped_robot_controller gazebo_sim.launch.py
ros2 run keyboard_input keyboard_input
ros2 run reference_trajectory_generator reference_trajectory_generator

# 列出所有 Gazebo 内部的跟pose话题
gz topic -l | grep pose

#列举ros2中带有imu名字的话题
ros2 topic list | grep imu

# 查看消息对应的内容
ros2 interface show sensor_msgs/msg/Imu

#安装数据曲线调试工具plotjuggler
sudo apt install ros-jazzy-plotjuggler-ros
#测试当前sdf语法是否会报错
gz sdf -k /home/laiyang/quadruped_robot/world/empty.sdf
#查看gazebo中是否有//model/quadruped_robot/link/base/pose该话题的数据输出
gz topic -e -t /model/quadruped_robot/link/base/pose

gz topic -i -t /model/quadruped_robot/link/base/pose
#gazebo发送topic数据，可以验证ros_gz_bridge是否工作
gz topic -t /model/quadruped_robot/link/base/pose -m gz.msgs.Pose -p 'name: "base"'