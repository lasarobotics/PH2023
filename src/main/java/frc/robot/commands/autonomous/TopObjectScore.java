// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class TopObjectScore extends SequentialCommandGroup {
  
  // construct a new auto command object
  public TopObjectScore(DriveSubsystem driveSubsystem) {
    AutoTrajectory topObjectScore2a = new AutoTrajectory(driveSubsystem, "top_object_score_2a");
    AutoTrajectory topObjectScore2b = new AutoTrajectory(driveSubsystem, "top_object_score_2b");
    AutoTrajectory topObjectScore2c = new AutoTrajectory(driveSubsystem, "top_object_score_2c");

    addCommands(
      topObjectScore2a.getCommandAndStop(),topObjectScore2b.getCommandAndStop(), topObjectScore2c.getCommandAndStop()
    );
  }
}
