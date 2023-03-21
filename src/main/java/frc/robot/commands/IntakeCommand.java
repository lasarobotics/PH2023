// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private ArmSubsystem m_armSubsystem;
  private DriveSubsystem m_driveSubsystem;
  private ArmState m_prevArmState;
  private CommandXboxController m_controller;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_armSubsystem = armSubsystem;
    this.m_driveSubsystem = driveSubsystem;
    this.m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_prevArmState = m_armSubsystem.getArmState();

    if (m_prevArmState == ArmState.Stowed) m_armSubsystem.setArmState(ArmState.Ground);
    
    m_intakeSubsystem.intake();
    m_driveSubsystem.enableSlowMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if (m_intakeSubsystem.isObjectPresent()) {
      m_controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      m_controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);

      if (m_prevArmState != ArmState.Middle && m_prevArmState != ArmState.High) {
        m_armSubsystem.setArmState(ArmState.Stowed);
        end(false);
      }
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

    if (interrupted && (m_prevArmState != ArmState.Middle && m_prevArmState != ArmState.High))
      m_armSubsystem.setArmState(ArmState.Stowed);

    m_intakeSubsystem.stop();
    m_driveSubsystem.disableSlowMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}