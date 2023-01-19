// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax wristMotor;
    private SparkMax rollerMotor;
    private ColorSensorV3 m_colorSensor;

    public Hardware(boolean isHardwareReal,
                    SparkMax wristMotor, 
                    SparkMax rollerMotor,
                    ColorSensorV3 m_colorSensor) {
      this.isHardwareReal = isHardwareReal;
      this.wristMotor = wristMotor;
      this.rollerMotor = rollerMotor;
      this.m_colorSensor = m_colorSensor;
    }
  }

  public enum GameObject {
    Cone(new Color(0.0, 0.0, 0.0)), Cube(new Color(0.0, 0.0, 0.0)); // TO-DO: Change color targets to be accurate
    
    public Color color;

    public GameObject(Color objectColor) {
      color = objectColor;
    }
  }

  private final SparkMax m_wristMotor;
  private final SparkMax m_rollerMotor;

  private final ColorMatch m_colorMatcher;

  /**
   * Create a new intake subsystem
   * @param intakeHardware Intake hardware
   */
  public IntakeSubsystem(Hardware intakeHardware) {

    this.m_colorMatcher = new ColorMatch();
    this.m_wristMotor = intakeHardware.wristMotor;
    this.m_rollerMotor = intakeHardware.rollerMotor;

    // Redundancy - restores motors to default (does so already within hardware PID)
    m_wristMotor.restoreFactoryDefaults();
    m_rollerMotor.restoreFactoryDefaults();

    // Set motors to break
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_rollerMotor.setIdleMode(IdleMode.kBrake);

    m_colorMatcher.addColorMatch(yellowTarget);
    m_colorMatcher.addColorMatch(purpleTarget);
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware intakeHardware = new Hardware(isHardwareReal,
                                           new SparkMax(Constants.IntakeHardware.WRIST_MOTOR_ID, MotorType.kBrushless),
                                           new SparkMax(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorType.kBrushless),
                                           new ColorSensorV3(I2C.Port.kOnboard)); // TO-DO: Change to correct port
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
   * Identify game object
   * @return GameObject, a Cone or Cube
   */
  public GameObject identifyObject() {
    Color detectedColor = m_colorMSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == GameObject.Cone.color) {
      return GameObject.Cone;
    }
    else if (match.color == GameObject.Cube.color) {
      return GameObject.Cube;
    }
    else {
      return null;
    }
  }

  /**
   * Read and return current color
   * @return Color, currently detected color by sensor
   */
  public Color readColor() {
    Color detectedColor = m_colorSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    return match.color;
  }

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
    m_colorSensor.close();
  }
}