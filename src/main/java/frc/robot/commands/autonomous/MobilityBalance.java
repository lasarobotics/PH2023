// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.AutoTrajectory;

public class MobilityBalance extends SequentialCommandGroup {
  private final boolean REVERSED = false;
  private final double MAX_VELOCITY = 3.0;
  private final double MAX_ACCELERATION = 4.0;
  
  // construct a new auto command object
  public MobilityBalance(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    List<PathPoint> path1 = List.of(
      new PathPoint(new Translation2d(0.0, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(3.2, 0.0), new Rotation2d())
    );
    List<PathPoint> path2 = List.of(
      new PathPoint(new Translation2d(3.2, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(5.5, 0.0), new Rotation2d())
    );
    List<PathPoint> path3 = List.of(
      new PathPoint(new Translation2d(5.5, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(3.0, 0.0), new Rotation2d())
    );

    addCommands(
      new StartEndCommand(() -> intakeSubsystem.outtake(), () -> intakeSubsystem.stop(), intakeSubsystem).withTimeout(1),
      new AutoTrajectory(driveSubsystem, path1, REVERSED, MAX_VELOCITY, MAX_ACCELERATION).getCommandAndStop(true),
      new AutoTrajectory(driveSubsystem, path2, REVERSED, MAX_VELOCITY / 2, MAX_ACCELERATION / 2).getCommandAndStop(),
      new AutoTrajectory(driveSubsystem, path3, !REVERSED, MAX_VELOCITY, MAX_ACCELERATION).getCommandAndStop(),
      new RunCommand(() -> driveSubsystem.autoBalance(), driveSubsystem)
    );
  }
}

