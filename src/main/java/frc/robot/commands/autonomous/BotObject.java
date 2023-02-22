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
public class BotObject extends SequentialCommandGroup {
  // create a new tool objects
  public BotObject(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    AutoTrajectory goToObject = new AutoTrajectory(driveSubsystem, "bot_object_1");

    addCommands(
      goToObject.getCommandAndStop(),
      new InstantCommand(() -> { intakeSubsystem.intake() }, intakeSubsystem))
    );
  }
}
