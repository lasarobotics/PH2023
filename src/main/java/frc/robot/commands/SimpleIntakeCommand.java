// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class SimpleIntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private CommandXboxController m_primaryController;
  private CommandXboxController m_secondaryController;

  /** Creates a new IntakeCommand. */
  public SimpleIntakeCommand(IntakeSubsystem intakeSubsystem, CommandXboxController primaryController, CommandXboxController secondaryController) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_primaryController = primaryController;
    this.m_secondaryController = secondaryController;

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
      m_primaryController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      m_primaryController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
      m_secondaryController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      m_secondaryController.getHID().setRumble(RumbleType.kRightRumble, 1.0);

    } else {
      m_primaryController.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
      m_primaryController.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
      m_secondaryController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
      m_secondaryController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_primaryController.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    m_primaryController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    m_secondaryController.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
    m_secondaryController.getHID().setRumble(RumbleType.kRightRumble, 0.0);


    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}