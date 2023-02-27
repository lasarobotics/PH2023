/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory {
  // Ramsete Command values
  private final double VOLTS_kS = 0.37608; 
  private final double VOLT_SECONDS_PER_METER_kV = 2.8367;
  private final double VOLT_SECONDS_SQUARED_PER_METER_kA = 0.23014;
  private final double kP = 1.0;
  private final double kD = 0; 
  private final boolean USE_ALLIANCE = true;

  DriveSubsystem m_driveSubsystem;
  PPRamseteCommand m_ramseteCommand;
  PathPlannerTrajectory m_trajectory;
  
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

    RamseteController ramseteController = new RamseteController();
    ramseteController.setEnabled(true);

    m_ramseteCommand = new PPRamseteCommand(
      m_trajectory, 
      m_driveSubsystem::getPose,
      ramseteController, 
      new SimpleMotorFeedforward(VOLTS_kS, VOLT_SECONDS_PER_METER_kV, VOLT_SECONDS_SQUARED_PER_METER_kA), 
      m_driveSubsystem.getKinematics(), 
      m_driveSubsystem::getWheelSpeeds, 
      new PIDController(kP, 0.0, kD), 
      new PIDController(kP, 0.0, kD), 
      m_driveSubsystem::autoTankDrive, 
      USE_ALLIANCE, 
      m_driveSubsystem
    );
  }

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints list of x, y coordinate pairs in trajectory
   * @param reversed whether the trajectory followed should be in reverse
   * @param maxVelocity Maximum velocity of robot during path (m/s)
   * @param maxAcceleration Maximum acceleration of robot during path (m/s^2)
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, List<PathPoint> waypoints, boolean reversed, double maxVelocity, double maxAcceleration) {
    this.m_driveSubsystem = driveSubsystem;

    m_trajectory = PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration), reversed, waypoints);

    RamseteController ramseteController = new RamseteController();
    ramseteController.setEnabled(true);

    m_ramseteCommand = new PPRamseteCommand(
      m_trajectory, 
      m_driveSubsystem::getPose,
      ramseteController, 
      new SimpleMotorFeedforward(VOLTS_kS, VOLT_SECONDS_PER_METER_kV, VOLT_SECONDS_SQUARED_PER_METER_kA), 
      m_driveSubsystem.getKinematics(), 
      m_driveSubsystem::getWheelSpeeds, 
      new PIDController(kP, 0.0, kD), 
      new PIDController(kP, 0.0, kD), 
      m_driveSubsystem::autoTankDrive, 
      USE_ALLIANCE, 
      m_driveSubsystem
    );
  }

  /**
   * Reset drive odometry to beginning of this path
   */
  private void resetOdometry() {
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommandAndStop() {
    return m_ramseteCommand.andThen(() -> {
            m_driveSubsystem.resetDrivePID();
            m_driveSubsystem.stop();
           });
  }

  /**
   * Get Ramsete command to run, resetting odometry first
   * @param isFirstPath true if path is the first one in autonomous
   * @return Ramsete command that will stop when complete
   */
  public Command getCommandAndStop(boolean isFirstPath) {
    if (isFirstPath) {
      return new InstantCommand(() -> resetOdometry())
                 .andThen(m_ramseteCommand)
                 .andThen(() -> {
                    m_driveSubsystem.resetDrivePID();
                    m_driveSubsystem.stop();
                 });
    } else return getCommandAndStop();
  }
  
  /**
   * Get Ramsete command to run
   * @return Ramsete command that does NOT stop when complete
   */
  public Command getCommand() {
    return m_ramseteCommand.andThen(() -> m_driveSubsystem.resetDrivePID());
  }

  /**
   * Get Ramsete command to run, resetting odometry first
   * @param isFirstPath true if path is first one in autonomous
   * @return Ramsete command that does NOT stop when complete
   */
  public Command getCommand(boolean isFirstPath) {
    if (isFirstPath) {
      return new InstantCommand(() -> resetOdometry())
                 .andThen(m_ramseteCommand)
                 .andThen(() -> m_driveSubsystem.resetDrivePID());
    } else return getCommand();  
  }
}