// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class MidPadObject extends SequentialCommandGroup {
	
  // construct a new auto command object
	public MidPadObject(DriveSubsystem driveSubsystem){
		AutoTrajectory midPadObject = new AutoTrajectory(driveSubsystem, "mid_pad_object_2");

		addCommands(
			midPadObject.getCommandAndStop()
		);
	}
}
