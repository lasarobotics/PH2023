/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory {
  // Ramsete Command values
  private final double VOLTS_kS = 0.1941; 
  private final double VOLT_SECONDS_PER_METER_kV = 2.8541;
  private final double VOLT_SECONDS_SQUARED_PER_METER_kA = 0.35776;
  private final double kP = 1;
  private final double kD = 0; 
  private final double kRamseteB = 2.0;
  private final double kRamseteZeta = 0.7;
  private final double MAX_VOLTAGE = 11.0;

  DriveSubsystem m_driveSubsystem;
  RamseteCommand m_ramseteCommand;
  Trajectory m_trajectory;
  
  /**
   * Create new path trajectory using PathPlanner path
   * @param driveSubsystem DriveSubsystem to drive the robot
   * @param pathName PathPlanner path name
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, String pathName) {
    this.m_driveSubsystem = driveSubsystem;

    m_trajectory = PathPlanner.loadPath(pathName, PathPlanner.getConstraintsFromPath(pathName));

    RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    ramseteController.setEnabled(true);

    m_ramseteCommand = new RamseteCommand(
      m_trajectory,
      driveSubsystem::getPose,
      ramseteController,
      m_driveSubsystem.getKinematics(),
      m_driveSubsystem::autoTankDrive,
      m_driveSubsystem 
    );
  }

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints list of x, y coordinate pairs in trajectory
   * @param isReversed whether the trajectory followed should be in reverse
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, Pose2d[] waypoints, boolean isReversed, double maxVelocity, double maxAcceleration) {
    this.m_driveSubsystem = driveSubsystem;
    
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(VOLTS_kS,
                                   VOLT_SECONDS_PER_METER_kV,
                                   VOLT_SECONDS_SQUARED_PER_METER_kA),
        m_driveSubsystem.getKinematics(),
        MAX_VOLTAGE
      );
    
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
    config.setKinematics(m_driveSubsystem.getKinematics());
    config.addConstraint(autoVoltageConstraint);
    config.setReversed(isReversed);
    
    List<Pose2d> waypointList = new ArrayList<Pose2d>();
    for (int i = 0; i < waypoints.length; i++) waypointList.add(waypoints[i]);

    m_trajectory = TrajectoryGenerator.generateTrajectory(waypointList, config);
    System.out.println("Trajectory: " + m_trajectory.toString());

    RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    ramseteController.setEnabled(true);

    m_ramseteCommand = new RamseteCommand(
      m_trajectory,
      driveSubsystem::getPose,
      ramseteController,
      m_driveSubsystem.getKinematics(),
      m_driveSubsystem::autoTankDrive,
      m_driveSubsystem 
    );
  }

  /**
   * Reset drive odometry to beginning of this path
   */
  public void resetOdometry() {
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommandAndStop() {
    return new InstantCommand(() -> resetOdometry())
               .andThen(m_ramseteCommand)
               .andThen(() -> {
                m_driveSubsystem.resetDrivePID();
                m_driveSubsystem.stop();
               });
  }
  
  /**
   * Get Ramsete command to run
   * @return Ramsete command that does NOT stop when complete
   */
  public Command getCommand() {
    return new InstantCommand(() -> resetOdometry())
               .andThen(m_ramseteCommand)
               .andThen(() -> {
                m_driveSubsystem.resetDrivePID();
               });
  }
}
