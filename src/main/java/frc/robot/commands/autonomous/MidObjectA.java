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
 * Goes to top middle object from middle starting position and picks the object up
 */
public class MidObjectA extends SequentialCommandGroup {

	/** 
   * Create an instance of MidObjectA auto command
   *
   * @param driveSubsystem Pass in instance of driveSubsystem
   * @param intakeSubsystem Pass in instance of intakeSubsystem
  */
  public MidObjectA(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    AutoTrajectory goToObject = new AutoTrajectory(driveSubsystem, "mid_objecta_1");

    addCommands(
      goToObject.getCommandAndStop(),
      new InstantCommand(() -> intakeSubsystem.intake(), intakeSubsystem)
    );
  }
}
