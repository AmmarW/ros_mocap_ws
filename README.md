# ros_mocap_ws
ROS workspace for Vicon and OptiTrack mocap streaming (Used with Clearpath platforms)
  
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

## Vicon Usage

1. Setup Shogun Live, calibrate cameras, and set the origin.
2. Enable "<subject_name>" under the 'Props' section.
3. Run the launch file. 
   ```bash
   roslaunch vicon_bridge vicon.launch
   # Ensure it is publishing to the `vicon/<subject_name>/<segment_name>` topic.
   ```

## OptiTrack Usage

1. In the Motive software, configure your rigid bodies.
2. Update `src/mocap_optitrack/config/mocap.yaml` to match the rigid body ID from Motive.
3. Run the launch file:
   ```bash
   roslaunch mocap_optitrack mocap.launch
   # or launch with RVIZ (if installed)
   roslaunch mocap_optitrack mocap.launch & rosrun rviz rviz
   ```

## Original Repositories & Broader ROS Support

For more information and support for other ROS versions, see the original sources:

- [OptiTrack: ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack)
- [Vicon: ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
