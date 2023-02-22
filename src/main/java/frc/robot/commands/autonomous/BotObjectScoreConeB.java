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
public class BotObjectScoreConeB extends SequentialCommandGroup {
	// create a new tool object
	public BotObjectScoreConeB(DriveSubsystem driveSubsystem){
		AutoTrajectory goToObject = new AutoTrajectory(driveSubsystem, "bot_object_1");
		AutoTrajectory scoreObject = new AutoTrajectory(driveSubsystem, "bot_object_score_2coneb");

		addCommands(
			goToObject.getCommandAndStop(),
			scoreObject.getCommandAndStop()
		);
	}
}
