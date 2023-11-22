# Proportional-Controller-Implementation-for-a-automotive-urdf-model-in-Gazebo-Rviz
Implemented a proportional controller for a car in ROS2

#Launch file Descriptions:
#display.launch

   - Launches robot+mounted lidar in a rviz/basic_display configuration in RViz
   - Used to check and verify visualization
   - Command to launch:('ros2 launch car_cad display.launch')

#gazebo.launch

   - Launches just the robot in a empty gazebo world
   - Used to verify robot spawning in gazebo
   - Command to launch:('ros2 launch car_cad gazebo.launch')

#teleop.launch

  - Launches robot+lidar in the competition_arena world, with gazebo sim paused
  - Used with car_teleop package to control the car using a keyboard
  - Command to launch:('roslaunch robot_urdf teleop.launch')
  - It also launches an rviz window which already where lidar scans are visualized
  - Unpause gazebo sim to solve error on RVIZ

#subscriber.launch

  - Launches robot+lidar in the empty world, with gazebo sim paused
  - Used with open_loop_controller package to make the robot run in a straight line
  - Command to launch:('ros2 launch car_cad subscriber.launch')

#How to build

  - cd to workspace folder and run ('colcon build')
  - if everything builds successfully (unless you have dependency issues), you can run any launch files
  - 
#How to Use

- Use display.launch (`ros2 launch car_cad display.launch`) to visualize robot_lidar in rviz, use the joint_state_publisher_gui to move nodes
- Use gazebo.launch (`ros2 launch car_cad gazebo.launch`) to spawn JUST THE ROBOT in gazebo
- For Teleop, run this package's teleop.launch file first (`ros2 launch car_cad teleop.launch`), once gazebo spawns the world, model, and controller, unpause gaz sim, and then open another terminal and type "roslaunch car_teleop teleop.launch".
- For Simple Pub-Sub, run the subscriber.launch file first (`ros2 launch car_cad subscriber.launch`), once everything spawns properly, unpause the sim, open another terminal, and run `ros2 launch open_loop_controller command_publisher.launch`. This should make the robot go in a straight line. To subscribe to the controller command topics, run `ros2 launch open_loop_controller command_subscriber.launch`
>>>>>>> origin/master
