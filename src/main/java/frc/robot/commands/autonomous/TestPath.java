// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.AutoTrajectory;

public class TestPath extends SequentialCommandGroup {

  HashMap<String, Command> eventMap = new HashMap<>();
  
  // construct a new auto command object
  public TestPath(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubystem) {

    AutoTrajectory testPath = new AutoTrajectory(driveSubsystem, "test_path");

    addCommands(
      testPath.getCommandAndStopWithEvents(true, eventMap)
    );
  }
}
