# F1TENTH AUTODRIVE for IROS 2024.
Working:- 

 1. Run the autodrive_f1tenth package using
`ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py`
or
`ros2 launch autodrive_f1tenth simulator_beingup_rviz.launch.py`

 2. For Making map:- 
  - Run slam_toolbox and nav2_bringup. Also run publish_odom.launch.py for publishing odom, because they require it and autodrive packages does not publishes it. Then save the map. pgm and yaml file will be saved. (Refer to commands.txt for more.)

 3. For making racing line follow these repos:- 
  - [Steven Gong](https://github.com/CL2-UWaterloo/f1tenth_ws/tree/main?tab=readme-ov-file)
 4. Run particle filter using `ros2 launch particle_filter localize.launch.py`
 5. Run pure_pursuit or stanley_avoidance for waypoint following.

Still yet to do:- 
Because of the less frequency of lidar data from the simulator, particle filters are not able to recognize the obstalces.
For some reason the car turns less. Needs some tuning, or checking on some parameters.

Learning:- 
 - How to use encoder ticks to publish odom data.
 - How transforms works.
 - It is necessary to make a /clock message type when working with simulators (got to know when making map, so had to make a publisher for /clock).