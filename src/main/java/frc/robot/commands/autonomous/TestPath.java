// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.utils.AutoTrajectory;

public class TestPath extends SequentialCommandGroup {

  HashMap<String, Command> eventMap = new HashMap<>();
  
  // construct a new auto command object
  public TestPath(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubystem) {
    eventMap.put(Constants.Auto.EVENT_MAP_INTAKE, new InstantCommand(() -> intakeSubsystem.intake()));
    eventMap.put(Constants.Auto.EVENT_MAP_OUTAKE, new InstantCommand(() -> intakeSubsystem.outake()));
    eventMap.put(Constants.Auto.EVENT_MAP_PRINT, new PrintCommand("It has crossed the threshold"));
    eventMap.put(Constants.Auto.EVENT_MAP_STOWED, new InstantCommand(() -> armSubystem.setArmState(ArmState.Stowed)));
    eventMap.put(Constants.Auto.EVENT_MAP_GROUND, new InstantCommand(() -> armSubystem.setArmState(ArmState.Ground)));
    eventMap.put(Constants.Auto.EVENT_MAP_MIDDLE, new InstantCommand(() -> armSubystem.setArmState(ArmState.Middle)));
    eventMap.put(Constants.Auto.EVENT_MAP_HIGH, new InstantCommand(() -> armSubystem.setArmState(ArmState.High)));

    AutoTrajectory testPath = new AutoTrajectory(driveSubsystem, "test_path");

    addCommands(
      testPath.getCommandAndStopWithEvents(eventMap)
    );
  }
}
