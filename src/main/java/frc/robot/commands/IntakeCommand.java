// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private CommandXboxController m_controller;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if (m_intakeSubsystem.isObjectPresent()) {
      m_controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      m_controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
    } else {
      m_controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
      m_controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);

    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}