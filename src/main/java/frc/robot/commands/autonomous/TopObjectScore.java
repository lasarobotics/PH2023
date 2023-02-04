// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopObjectScore extends SequentialCommandGroup {
  /** Creates a new TopObjectScore. */
  public TopObjectScore(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoTrajectory topObjectScore2a = new AutoTrajectory(driveSubsystem, "top_object_score_2a");
    AutoTrajectory topObjectScore2b = new AutoTrajectory(driveSubsystem, "top_object_score_2b");
    AutoTrajectory topObjectScore2c = new AutoTrajectory(driveSubsystem, "top_object_score_2c");
    addCommands(
      topObjectScore2a.getCommandAndStop(),topObjectScore2b.getCommandAndStop(), topObjectScore2c.getCommandAndStop()
    );
  }
}
