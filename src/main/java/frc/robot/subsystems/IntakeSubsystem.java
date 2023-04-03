// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax rollerMotor;
    private SparkMaxLimitSwitch objectPresenceDetector;
    private SparkMaxLimitSwitch objectDifferentiator;

    public Hardware(boolean isHardwareReal,
        SparkMax rollerMotor,
        SparkMaxLimitSwitch objectPresenceDetector,
        SparkMaxLimitSwitch objectDifferentiator) {
      this.isHardwareReal = isHardwareReal;
      this.rollerMotor = rollerMotor;
      this.objectPresenceDetector = objectPresenceDetector;
      this.objectDifferentiator = objectDifferentiator;
    }
  }

  public enum GameObject {
    Cone, Cube
  }

  private SparkMax m_rollerMotor;
  private SparkMaxLimitSwitch m_objectPresenceDetector;
  private SparkMaxLimitSwitch m_objectDifferentiator;

  private double m_intakeSpeed;
  private double m_outtakeSpeed;

  private final int ROLLER_CURRENT_LIMIT = 30;
  private final double ROLLER_STOPPED_THRESHOLD = 0.25;
  private final double ROLLER_CREEP = 0.1;

  /**
   * Create a new intake subsystem
   * 
   * @param intakeHardware Intake hardware
   * @param intakeSpeed Intake speed (duty cycle, positive)
   * @param outtakeSpeed Outtake Speed (duty cycle, negative)
   */
  public IntakeSubsystem(Hardware intakeHardware, double intakeSpeed, double outtakeSpeed) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_objectPresenceDetector = intakeHardware.objectPresenceDetector;
    this.m_objectDifferentiator = intakeHardware.objectDifferentiator;
    this.m_intakeSpeed = (intakeSpeed > 0.0) ? intakeSpeed : intakeSpeed * -1;
    this.m_outtakeSpeed = (outtakeSpeed < 0.0) ? outtakeSpeed : outtakeSpeed * -1;

    // Reset motors to default
    m_rollerMotor.restoreFactoryDefaults();

    // Set motors to brake
    m_rollerMotor.setIdleMode(IdleMode.kCoast);

    m_rollerMotor.setSmartCurrentLimit(ROLLER_CURRENT_LIMIT);

    // Inver roller motor
    m_rollerMotor.setInverted(true);

    // Reset presence detection limit switches to default
    m_objectPresenceDetector.enableLimitSwitch(true);
    m_objectDifferentiator.enableLimitSwitch(false);

    // Only do this stuff if hardware is real
    if (intakeHardware.isHardwareReal) {
    }
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    SparkMax rollerMotor = new SparkMax(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorType.kBrushless);
    Hardware intakeHardware = new Hardware(
      isHardwareReal,
      rollerMotor,
      rollerMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed),
      rollerMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)
    );
    
    return intakeHardware;
  }

  /**
   * Intake game object
   */
  public void intake() {
    m_objectPresenceDetector.enableLimitSwitch(true);
    m_rollerMotor.set(m_intakeSpeed, ControlType.kDutyCycle);
  }

  /**
   * Outtake game object
   */
  public void outtake() {
    m_objectPresenceDetector.enableLimitSwitch(false);
    m_rollerMotor.set(m_outtakeSpeed, ControlType.kDutyCycle);
  }

  /**
   * Identifies whether a game object is present
   * 
   * @return true if object is present
   */
  public boolean isObjectPresent() {
    return m_rollerMotor.getEncoderVelocity() < ROLLER_STOPPED_THRESHOLD;
  }

  /**
   * Identify game object
   * 
   * @return Game object, null if unidentified
   */
  public GameObject identifyObject() {
    if (isObjectPresent()) {
      if (m_objectDifferentiator.isPressed())
        return GameObject.Cone;
      else
        return GameObject.Cube;
    }
    return null;
  }

  /**
   * Stops motor
   */
  public void stop() {
    m_rollerMotor.set(ROLLER_CREEP);
  }

  @Override
  public void close() {
    m_rollerMotor.close();
  }
}