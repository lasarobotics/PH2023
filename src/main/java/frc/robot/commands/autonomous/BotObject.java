// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.AutoTrajectory;

/** 
 * Goes to bottom object from bottom starting position and picks the object up
 */
public class BotObject extends SequentialCommandGroup {

  /** 
   * Create an instance of BotObject auto command
   *
   * @param driveSubsystem Pass in instance of driveSubsystem
   * @param intakeSubsystem Pass in instance of intakeSubsystem
  */
  public BotObject(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    AutoTrajectory goToObject = new AutoTrajectory(driveSubsystem, "bot_object_1");

    addCommands(
      goToObject.getCommandAndStop(),
      new InstantCommand(() ->  intakeSubsystem.intake(), intakeSubsystem)
    );
  }
}
