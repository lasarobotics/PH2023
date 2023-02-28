// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class BotScoreConeB extends SequentialCommandGroup {
  
  // construct a new auto command object
  public BotScoreConeB(DriveSubsystem driveSubsystem, HashMap<String, Command> eventMap) {

    AutoTrajectory testPath = new AutoTrajectory(driveSubsystem, "bot_scoreConeA_object");

    addCommands(
      testPath.getCommandAndStopWithEvents(true, eventMap)
    );
  }
}
