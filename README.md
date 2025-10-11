# ros_mocap_ws
ROS workspace for Vicon and OptiTrack mocap streaming (For use with Clearpath platforms)
  
## Build Instructions

Supported only on **ROS1 Noetic**.

Build:
```bash
cd ~/ros_mocap_ws
catkin build
```

Source workspace:
```bash
source ~/ros_mocap_ws/devel/setup.bash  # one-time 
echo "source ~/ros_mocap_ws/devel/setup.bash" >> ~/.bashrc # permanent
```

## OptiTrack Usage

Once built and sourced, you can start OptiTrack streaming with:

```bash
roslaunch mocap_optitrack mocap.launch
# or launch with RVIZ (if installed)
roslaunch mocap_optitrack mocap.launch & rosrun rviz rviz
```

## Original Repositories & Broader ROS Support

For more information and support for other ROS versions, see the original sources:

- [OptiTrack: ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack)
- [Vicon: ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
