// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax shoulderMasterMotor, shoulderSlaveMotor;
    private SparkMax elbowMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax shoulderMasterMotor, 
                    SparkMax shoulderSlaveMotor,
                    SparkMax elbowMotor) {
      this.isHardwareReal = isHardwareReal;
      this.shoulderMasterMotor = shoulderMasterMotor;
      this.shoulderSlaveMotor = shoulderSlaveMotor;
      this.elbowMotor = elbowMotor;
    }
  }

  /**
   * Arm States
   */
  public enum ArmState {
    Stowed(-1.0, -1.0),
    Ground(-21.0, -60.0),
    Middle(-93.0, -60.0),
    High(-115.0, -55.0);

    public final double shoulderAngle;
    public final double elbowAngle;
    private ArmState(double shoulderAngle, double elbowAngle) {
      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
    }
  }

  private SparkMax m_shoulderMasterMotor;
  private SparkMax m_shoulderSlaveMotor;
  private SparkMax m_elbowMotor;

  private SparkPIDConfig m_shoulderMotionConfig;
  private SparkPIDConfig m_shoulderPositionConfig;
  private SparkPIDConfig m_elbowMotionConfig;
  private SparkPIDConfig m_elbowPositionConfig;

  private final int MOTION_CONFIG_PID_SLOT = 0;
  private final int POSITION_CONFIG_PID_SLOT = 1;

  private ArmState m_currentState;

  /**
   * Create an instance of ArmSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param armHardware Hardwave devices required by arm
   * @param shoulderConfigs Paired PID config for arm shoulder (First -> Motion Config, Second -> Position Config)
   * @param elbowConfigs Paired PID config for arm elbow (First -> Motion Config, Second -> Position Config)
   */

  public ArmSubsystem(Hardware armHardware, Pair<SparkPIDConfig, SparkPIDConfig> shoulderConfigs, Pair<SparkPIDConfig, SparkPIDConfig> elbowConfigs) {
    this.m_shoulderMasterMotor = armHardware.shoulderMasterMotor;
    this.m_shoulderSlaveMotor = armHardware.shoulderSlaveMotor;
    this.m_elbowMotor = armHardware.elbowMotor;

    this.m_shoulderMotionConfig = shoulderConfigs.getFirst();
    this.m_shoulderPositionConfig = shoulderConfigs.getSecond();
    this.m_elbowMotionConfig = elbowConfigs.getFirst();
    this.m_elbowPositionConfig = elbowConfigs.getSecond();

    m_currentState = ArmState.Stowed;


    
    // Set all arm motors to brake
    m_shoulderMasterMotor.setIdleMode(IdleMode.kCoast);
    m_shoulderSlaveMotor.setIdleMode(IdleMode.kCoast);
    m_elbowMotor.setIdleMode(IdleMode.kCoast);

    // Make slave follow master
    m_shoulderSlaveMotor.follow(m_shoulderMasterMotor);

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // Initialize PID
      m_shoulderMotionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getEncoder(), false, false, MOTION_CONFIG_PID_SLOT);
      m_elbowMotionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getEncoder(), false, false, MOTION_CONFIG_PID_SLOT);

      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getEncoder(), false, false, POSITION_CONFIG_PID_SLOT);
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getEncoder(), false, false, POSITION_CONFIG_PID_SLOT);

      // Reset encoders
      m_shoulderMasterMotor.resetEncoder();
      m_elbowMotor.resetEncoder();
    }
  }


   /**
   * Initialize hardware devices for drive subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(isHardwareReal,
                                        new SparkMax(Constants.ArmHardware.ARM_SHOULDER_MASTER_MOTOR_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_SHOULDER_SLAVE_MOTOR_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_ELBOW_MOTOR_ID, MotorType.kBrushless));
    return armHardware;
  }

  /**
   * Makes the arm hold position
   */
  private void holdPosition(boolean shoulderMotionComplete, boolean elbowMotionComplete) {
    // Set shoulder and elbow positions
    if (shoulderMotionComplete)
      m_shoulderMasterMotor.set(m_currentState.shoulderAngle, ControlType.kPosition, 0.0, ArbFFUnits.kPercentOut, POSITION_CONFIG_PID_SLOT);
    if (elbowMotionComplete)
      m_elbowMotor.set(m_currentState.elbowAngle, ControlType.kPosition, 0.0, ArbFFUnits.kPercentOut, POSITION_CONFIG_PID_SLOT);
  }

  /**
   * Check if shoulder motion is complete
   * @return True if shoulder motion is complete
   */
  public boolean isShoulderMotionComplete() {
    return Math.abs(m_shoulderMasterMotor.get()) <= Arm.MOTOR_END_VELOCITY_THRESHOLD && m_shoulderMasterMotor.getEncoderPosition() == m_currentState.shoulderAngle;
  }

  /**
   * Check if motion is complete
   * @return True if elbow motion is complete
   */
  public boolean isElbowMotionComplete() {
    return Math.abs(m_elbowMotor.get()) <= Arm.MOTOR_END_VELOCITY_THRESHOLD && m_elbowMotor.getEncoderPosition() == m_currentState.elbowAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Hold position if motion is complete
    holdPosition(isShoulderMotionComplete(), isElbowMotionComplete());

    System.out.println("Shoulder position: " + m_shoulderMasterMotor.getEncoderPosition());
    System.out.println("Elbow position: " + m_elbowMotor.getEncoderPosition());
  }

  /**
   * Set desired arm state
   * @param armState Arm state, which includes shoulder and elbow position
   */
  public void setArmState(ArmState armState) {
    // Update current state
    m_currentState = armState;

    m_shoulderMasterMotor.set(m_currentState.shoulderAngle, ControlType.kSmartMotion, 0.0, ArbFFUnits.kPercentOut, MOTION_CONFIG_PID_SLOT);
    m_elbowMotor.set(m_currentState.elbowAngle, ControlType.kSmartMotion, 0.0, ArbFFUnits.kPercentOut, MOTION_CONFIG_PID_SLOT);
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
    m_shoulderMasterMotor.close();
    m_elbowMotor.close();
  }
}
