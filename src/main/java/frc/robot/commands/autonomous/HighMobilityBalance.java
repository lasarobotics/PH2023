// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.AutoTrajectory;

public class HighMobilityBalance extends SequentialCommandGroup {
  
  // construct a new auto command object
  public HighMobilityBalance(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    List<PathPoint> path1 = List.of(
      new PathPoint(new Translation2d(0.0, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(3.2, 0.0), new Rotation2d())
    );
    List<PathPoint> path2 = List.of(
      new PathPoint(new Translation2d(3.2, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(6.0, 0.0), new Rotation2d())
    );
    List<PathPoint> path3 = List.of(
      new PathPoint(new Translation2d(6.0, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(2.8, 0.0), new Rotation2d())
    );

    addCommands(
      new ArmCommand(ArmState.High, armSubsystem),
      new WaitCommand(0.3),
      new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem).withTimeout(0.75),
      new InstantCommand(() -> armSubsystem.setArmState(ArmState.Stowed), armSubsystem),
      new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem),
      new AutoTrajectory(driveSubsystem, path1, false, 3.0, 4.0).getCommandAndStop(true),
      new AutoTrajectory(driveSubsystem, path2, false, 1.0, 1.0).getCommandAndStop(),
      // new TurnCommand(driveSubsystem),
      new AutoTrajectory(driveSubsystem, path3, true, 3.0, 3.0).getCommandAndStop(),
      new RunCommand(() -> driveSubsystem.autoBalance(), driveSubsystem)
    );
  }
}


