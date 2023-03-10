// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTrajectory;

public class Balance extends SequentialCommandGroup {
  
  // construct a new auto command object
  public Balance(DriveSubsystem driveSubsystem) {
    List<PathPoint> waypoints = List.of(
      new PathPoint(new Translation2d(0.0, 0.0), new Rotation2d()),
      new PathPoint(new Translation2d(2.3, 0.0), new Rotation2d())
    );

    addCommands(
      new AutoTrajectory(driveSubsystem, waypoints, true, 0.5, 0.5).getCommandAndStop(),
      new InstantCommand(() -> driveSubsystem.autoBalance())
    );
  }
}


