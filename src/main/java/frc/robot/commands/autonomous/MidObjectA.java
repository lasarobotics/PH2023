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
public class MidObjectA extends SequentialCommandGroup {
  /** Creates a new TopObject. */
  public MidObjectA(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    AutoTrajectory midObjectA1 = new AutoTrajectory(driveSubsystem, "mid_objecta_1");
    AutoTrajectory midObjectA2 = new AutoTrajectory(driveSubsystem, "mid_objecta_1b");
    addCommands(
      midObjectA1.getCommandAndStop(),
      midObjectA2.getCommandAndStop()
    );
  }
}
