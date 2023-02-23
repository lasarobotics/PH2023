// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

/** 
 * Goes to top middle object from middle starting position and picks the object up, then scores on middle middle cube shelf
 */
public class MidObjectAScoreCube extends SequentialCommandGroup {

	/** 
   * Create an instance of MidObjectAScoreCube auto command
   *
   * @param driveSubsystem Pass in instance of driveSubsystem
   * @param intakeSubsystem Pass in instance of intakeSubsystem
  */
	public MidObjectAScoreCube(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem){
		AutoTrajectory goToObject = new AutoTrajectory(driveSubsystem, "mid_objecta_1");
		AutoTrajectory scoreObject = new AutoTrajectory (driveSubsystem, "mid_objecta_score_2cube");

		addCommands(
			goToObject.getCommandAndStop(),
			new InstantCommand(() -> { intakeSubsystem.intake() }, intakeSubsystem)),
			scoreObject.getCommandAndStop()
		);
	}
}
