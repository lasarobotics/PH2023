# FRC 2023
Team 418's 2023 robot code. The robot's code is written in Java and is based off of WPILib's Java command based control system.

The code is organised into several subsystems, each responsible for a different aspect of the robot's functionality. This document explains this code.

## [Subsystems](src/main/java/frc/robot/subsystems)
### [Drive Subsystem](src/main/java/frc/robot/subsystems/DriveSubsystem.java)
The drive subsystem controls the drivetrain of the robot, which is a 6-wheel differential drive. Each side of the robot is driven by a set of 2 NEO brushless motors, providing ample power for our robot.
The NEO also provides the benefit of having a built-in encoder to accuately measure wheel rotations. The motors are connected to the wheels by a gearbox with a 10.71:1 gear ratio.
We also implement a closed loop control system for our drivetrain, using the yaw axis gyroscope on the navX2 IMU for feedback. This allows the driver to drive perfectly straight, as well as allowing the robot to maintain orientation when being hit by other robots, driving over rough terrain, etc. This closed loop control includes a rudimentary traction control system, allowing the robot to limit wheel slip. The IMU is also used to aid in automatically balancing the robot in the pitch axis.
Lastly, the drivetrain also implements odometry and VSLAM (Visual Simultaneous Localization And Mapping) for accurate pose estimation, which allows the robot to use the IMU, encoder, and camera data to accuately determine its position on the field, a very useful feature.

Key points:
* 6 wheel differential drive
* Powered by 4 NEO brushless motors
* Closed loop drive control system with traction control
* Odometry for accurate positioning during autonomous and teleop