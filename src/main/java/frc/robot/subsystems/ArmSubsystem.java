// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax shoulderMotor, elbowMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax shoulderMotor, 
                    SparkMax elbowMotor) {
      this.isHardwareReal = isHardwareReal;
      this.shoulderMotor = shoulderMotor;
      this.elbowMotor = elbowMotor;
    }
  }

  public enum ArmState {
    Stowed(0.0, 0.0),
    Ground(0.0, 0.0),
    Middle(0.0, 0.0),
    High(0.0, 0.0);

    public final double x;
    public final double y;
    private ArmState(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  private SparkMax m_shoulderMotor, m_elbowMotor;

  private SparkPIDConfig m_shoulderConfig;
  private SparkPIDConfig m_elbowConfig;

  private ArmState m_currentState;

  /**
   * Create an instance of ArmSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param armHardware Hardwave devices required by arm
   * @param shoulderConfig PID config for arm shoulder
   * @param elbowConfig PID config for arm elbow
   */
  public ArmSubsystem(Hardware armHardware, SparkPIDConfig shoulderConfig, SparkPIDConfig elbowConfig) {
    this.m_shoulderMotor = armHardware.shoulderMotor;
    this.m_elbowMotor = armHardware.elbowMotor;
    m_currentState = ArmState.Stowed;

    // Set all arm motors to brake
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // initialize PID
      m_shoulderConfig.initializeSparkPID(m_shoulderMotor, m_shoulderMotor.getAlternateEncoder());
      m_elbowConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAnalog(Mode.kAbsolute));
    }
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(isHardwareReal,
                                        new SparkMax(Constants.ArmHardware.ARM_PIVOT_SHOULDER_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_PIVOT_ELBOW_ID, MotorType.kBrushless));
    return armHardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set desired arm state
   * @param armState Arm state, which includes pivot and telescope position
   */
  public void setArmState(ArmState armState) {
    // Update current state
    m_currentState = armState;

    // Set telescope and pivot positions
    m_shoulderMotor.set(0.0, ControlType.kSmartMotion);
    m_elbowMotor.set(0.0, ControlType.kSmartMotion);
  }

  /**
   * Get current arm state
   * @return current state
   */
  public ArmState getArmState() {
    return m_currentState;
  }

  @Override
  public void close() {
    m_shoulderMotor.close();
    m_elbowMotor.close();
  }
}
