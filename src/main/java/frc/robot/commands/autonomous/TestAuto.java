// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class TestAuto extends SequentialCommandGroup {
  
  // construct a new auto command object
  public TestAuto(DriveSubsystem driveSubsystem) {
    Pose2d waypoints[] = {
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      new Pose2d(2.0, 0.0, new Rotation2d(0.0))
    };

    addCommands(
      new AutoTrajectory(driveSubsystem, waypoints, false, 0.5, 0.5).getCommandAndStop()
    );
  }
}
