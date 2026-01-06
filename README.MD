# NokovToPx4 使用教程
<font size=4>**基于ubuntu22.04 ROS2_Humble**</font>

<font size=4> 本包是将NOKOV数据发布给PX4飞控作室内定位用</font>

## 1.基础配置
安装所有必须的包<px4_msg><geometry_msgs><vrpn_client_ros>到工作空间中
```
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/NOKOV-MOCAP/vrpn_client_ros.git
```
同样git clone本包，全部装完如下所示：
![相对路径](.\\picutre\\1.png "所有包")
## 2.配置相关参数后编译
首先对本包config/params.yaml的参数配置
```
/nokov_to_px4:
  ros__parameters:
    nokov_topic: /px/px4 #动捕的话题，根据设置的刚体名称修改
    px4_topic: /fmu/in/vehicle_visual_odometry #飞控的话题
```
其次对于vrpn_client_ros/config下的参数设置IP地址
```
vrpn_client_node:
  ros__parameters:
    server: 192.168.5.32 #主要修改此参数，依据实际参数确定
    port: 3883
    frame_id: "world"
    use_server_time: false
    refresh_tracker_frequency: 0.2
    update_frequency: 60.0
```
返回工作空间目录下编译
```
colcon build
```
## 3.运行
编译成功后source并运行（要用MicroXRCEAgent连接上飞控）
```
ros2 launch nokov_to_px4 run.launch.py
```
如果发布话题改变又不想重新编译可以
```
ros2 launch nokov_to_px4 run2.launch.py nokov_topic:=***** px4_topic:=*****
其中*****为话题名
```
运行结果如下：
![相对路径](.\\picutre\\2.png "运行结果")
订阅飞机里程计看到数据与动捕数据一致：
![相对路径](.\\picutre\\3.png "飞机数据")

## 4.作者
<font size=3>Xzhou </font>
 homepage [Github](https://github.com/Zhouxiang1027) / [Gitee](https://gitee.com/Xzhou9207)
本项目仅供学习参考，严禁用于商业行为。

Thanks for Open [contirbutor](https://github.com/SaxionMechatronics/mocap_px4_bridge)
