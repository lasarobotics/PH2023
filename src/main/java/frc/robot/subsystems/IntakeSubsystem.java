// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax wristMotor;
    private SparkMax rollerMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax wristMotor, 
                    SparkMax rollerMotor) {
      this.isHardwareReal = isHardwareReal;
      this.wristMotor = wristMotor;
      this.rollerMotor = rollerMotor;
    }
  }

  private final SparkMax m_wristMotor;
  private final SparkMax m_rollerMotor;

  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_wristMotor = intakeHardware.wristMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;

    // Idk if I need to do this stuff
    m_wristMotor.restoreFactoryDefaults();
    m_rollerMotor.restoreFactoryDefaults();

    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_rollerMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware intakeHardware = new Hardware(isHardwareReal,
                                           new SparkMax(Constants.IntakeHardware.WRIST_MOTOR_PORT, MotorType.kBrushless),
                                           new SparkMax(Constants.IntakeHardware.ROLLER_MOTOR_PORT, MotorType.kBrushless));
    return intakeHardware;
  }

  public void intake() {
    m_rollerMotor.set(Constants.IntakeHardware.SPIN_MOTOR_SPEED);
  }

  public void outake() {
    m_rollerMotor.set(-Constants.IntakeHardware.SPIN_MOTOR_SPEED);
  }

  public void stop() {
    m_rollerMotor.stopMotor();
  }
}