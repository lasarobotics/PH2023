// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax clawMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax clawMotor) {
      this.isHardwareReal = isHardwareReal;
      this.clawMotor = clawMotor;
    }
  }
  private SparkMax m_clawMotor;

  private final int OPEN_POSITION = 0;
  private final int CLOSE_POSITION = 1;

  /**
   * Create a new intake subsystem
   * @param intakeHardware Intake hardware
   */
  public IntakeSubsystem(Hardware intakeHardware, SparkPIDConfig clawConfig) {
    this.m_clawMotor = intakeHardware.clawMotor;

    // Reset motors to default
    m_clawMotor.restoreFactoryDefaults();

    // Set motors to break
    m_clawMotor.setIdleMode(IdleMode.kBrake);

    if(intakeHardware.isHardwareReal) {
      clawConfig.initializeSparkPID(m_clawMotor, m_clawMotor.getEncoder());

      // Set conversion factor
      double conversionFactor = 360;
      m_clawMotor.getEncoder().setPositionConversionFactor(conversionFactor);
    }
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware intakeHardware = new Hardware(isHardwareReal, new SparkMax(Constants.IntakeHardware.WRIST_MOTOR_ID, MotorType.kBrushless));
    return intakeHardware;
  }

  /**
   * Intake game object
   */
  public void openClaw() {
    m_clawMotor.set(OPEN_POSITION, ControlType.kSmartMotion);
  }

  /**
   * Outtake game object
   */
  public void closeClaw() {
    m_clawMotor.set(CLOSE_POSITION, ControlType.kSmartMotion);
  }

  /**
   * Stops motor
   */
  public void stop() {
    m_clawMotor.stopMotor();
  }

  @Override
  public void close() {
    m_clawMotor.close();
  }
}