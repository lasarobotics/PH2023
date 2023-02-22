// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax wristMotor;
    private SparkMax rollerMotor;
    private DigitalInput objectPresenceDetector;
    private DigitalInput objectDifferentiator;

    public Hardware(boolean isHardwareReal,
                    SparkMax wristMotor, 
                    SparkMax rollerMotor,
                    DigitalInput objectPresenceDetector,
                    DigitalInput objectDifferentiator) {
      this.isHardwareReal = isHardwareReal;
      this.wristMotor = wristMotor;
      this.rollerMotor = rollerMotor;
      this.objectPresenceDetector = objectPresenceDetector;
      this.objectDifferentiator = objectDifferentiator;
    }
  }

  // TODO: Change color targets to be accurate
  public enum GameObject {
    Cone,
    Cube
  }

  private SparkMax m_wristMotor;
  private SparkMax m_rollerMotor;
  private DigitalInput m_objectPresenceDetector;
  private DigitalInput m_objectDifferentiator;

  /**
   * Create a new intake subsystem
   * @param intakeHardware Intake hardware
   */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_wristMotor = intakeHardware.wristMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_objectPresenceDetector = intakeHardware.objectPresenceDetector;
    this.m_objectDifferentiator = intakeHardware.objectDifferentiator;

    // Reset motors to default
    m_wristMotor.restoreFactoryDefaults();
    m_rollerMotor.restoreFactoryDefaults();

    // Set motors to break
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_rollerMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware intakeHardware = new Hardware(isHardwareReal,
                                           new SparkMax(Constants.IntakeHardware.WRIST_MOTOR_ID, MotorType.kBrushless),
                                           new SparkMax(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorType.kBrushless),
                                           new DigitalInput(Constants.IntakeHardware.PRESENCE_SENSOR_PORT),
                                           new DigitalInput(Constants.IntakeHardware.DIFFERENTIATOR_SENSOR_PORT));
    return intakeHardware;
  }

  /**
   * Intake game object
   */
  public void intake() {
    m_rollerMotor.set(+Constants.Intake.SPIN_MOTOR_SPEED, ControlType.kDutyCycle);
  }

  /**
   * Outtake game object
   */
  public void outake() {
    m_rollerMotor.set(-Constants.Intake.SPIN_MOTOR_SPEED, ControlType.kDutyCycle);
  }

  /**
  * Identifies whether a game object is present
  * @return boolean
  */
  public boolean isObjectPresent() {
    return m_objectPresenceDetector.get(); // Circuit is open, object is present
  }

  /**
   * Identify game object
   * @return Game object, null if unidentified
   */
  public GameObject identifyObject() {
    if (isObjectPresent()) {
      if (m_objectDifferentiator.get()) // Circuit is open, whatever object the differentiator detects is present
        return GameObject.Cone; // Cone for now - SUBJECT TO CHANGE
      else
        return GameObject.Cube;
    }

    return null;
  }

  /**
   * Read and return current color
   * @return Color, currently detected color by sensor
   */

  /**
   * Stops motor
   */
  public void stop() {
    m_rollerMotor.stopMotor();
  }

  @Override
  public void close() {
    m_wristMotor.close();
    m_rollerMotor.close();
  }
}
