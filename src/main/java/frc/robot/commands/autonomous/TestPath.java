// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class TestPath extends SequentialCommandGroup {
  public TestPath(DriveSubsystem driveSubsystem) {
    AutoTrajectory testPath = new AutoTrajectory(driveSubsystem, "test_path");

    addCommands(
      testPath.getCommandAndStop(true)
    );
  }
}
