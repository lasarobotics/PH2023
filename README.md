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

## [Arm Subsystem](https://github.com/lasarobotics/PH2023/blob/master/src/main/java/frc/robot/subsystems/ArmSubsystem.java)

This arm subsystem consists of a virtual four-bar lift mechanism. The arm extending from the vertical post of the robot has two degrees of rotation: a smaller "wrist" at the end of the elongated component of the arm (dubbed "arm"). We implement two proportional-integral-derivative (PID) controllers for each segment of the arm controlling motion to a given position and remaining there. The moment to switch between the two controllers is dependent on both the velocity and position of the arm component. In general, these PID controlers employ a feedback to contoniuously calculate error and correct for it. To correct for error due to gravity, a feed-forward mechanism adjusts for the vertical component. To gain even greater dexterity of the motor positions and velocities, we use motion profiling to outline a position, velocity, and acceleration profile to be followed. With this, we can define any given movement as a series of steps, which allows for smoother and more accurate motions.

To construct this mechanism, we use a set of three brushless NEO motors with absolute encoders, which allows for tracking position more accurately and eliminating the necessity to reset the arm after each match. We use SparkMax motor controllers for motor motion management.

Keypoints:
- Virtual four bar with two degrees of movement (arm and wrist)
- PID controllers handling position and motion for each part of the arm, correcting for error due to gravity and other factors
- Switching between the two PID controllers based on position and velocity
- Motion profiling to gain greater dexterity of motor position and velocity.

## [Vision Subsystem](https://github.com/lasarobotics/PH2023/blob/master/src/main/java/frc/robot/subsystems/VisionSubsystem.java)
- 
## [Intake Subsystem](https://github.com/lasarobotics/PH2023/blob/master/src/main/java/frc/robot/subsystems/IntakeSubsystem.java)