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
public class MidObjectBScore extends SequentialCommandGroup {
	// create a new tool object
	public MidObjectBScore(DriveSubsystem driveSubsystem){
		AutoTrajectory midObjectB1 = new AutoTrajectory(driveSubsystem, "mid_objectb_score_2a");
		AutoTrajectory midObjectB2 = new AutoTrajectory(driveSubsystem, "mid_objectb_score_2b");
		AutoTrajectory midObjectB3 = new AutoTrajectory (driveSubsystem, "mid_objectb_score_2c");

		addCommands(
      midObjectB1.getCommandAndStop(), 
      midObjectB2.getCommandAndStop(), 
      midObjectB3.getCommandAndStop()
		);
	}
}
