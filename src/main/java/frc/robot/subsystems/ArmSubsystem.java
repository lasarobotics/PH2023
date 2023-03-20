// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    Stowed(+0.905, +0.580),
    Ground(+0.840, +0.305),
    Middle(+0.700, +0.280),
    High(+0.600, +0.020);

    public final double shoulderPosition;
    public final double elbowPosition;

    private ArmState(double shoulderPosition, double elbowPosition) {
      this.shoulderPosition = shoulderPosition;
      this.elbowPosition = elbowPosition;
    }
  }

  public enum ArmDirection {
    Up, Down, None;

    public static ArmDirection getArmDirection(ArmState from, ArmState to) {
      int diff = from.ordinal() - to.ordinal();
      return diff < 0 ? Down : diff < 0 ? Up : None;
    }
  }

  private SparkMax m_shoulderMasterMotor;
  private SparkMax m_shoulderSlaveMotor;
  private SparkMax m_elbowMotor;

  private ProfiledPIDController m_shoulderMotionConfig;
  private SparkPIDConfig m_shoulderPositionConfig;
  private ProfiledPIDController m_elbowMotionConfig;
  private SparkPIDConfig m_elbowPositionConfig;

  private final double CONVERSION_FACTOR = 360.0;
  private final double SHOULDER_FF = 0.011;
  private final double ELBOW_FF = 0.005;

  private final int MOTION_CONFIG_PID_SLOT = 0;
  private final int POSITION_CONFIG_PID_SLOT = 1;

  private ArmState m_currentArmState;
  private ArmDirection m_currentArmDirection;

  /**
   * Create an instance of ArmSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param armHardware     Hardwave devices required by arm
   * @param shoulderConfigs Paired PID config for arm shoulder (First -> Motion
   *                        Config, Second -> Position Config)
   * @param elbowConfigs    Paired PID config for arm elbow (First -> Motion
   *                        Config, Second -> Position Config)
   */

  public ArmSubsystem(Hardware armHardware, Pair<ProfiledPIDController, SparkPIDConfig> shoulderConfigs,
      Pair<ProfiledPIDController, SparkPIDConfig> elbowConfigs) {
    this.m_shoulderMasterMotor = armHardware.shoulderMasterMotor;
    this.m_shoulderSlaveMotor = armHardware.shoulderSlaveMotor;
    this.m_elbowMotor = armHardware.elbowMotor;

    this.m_shoulderMotionConfig = shoulderConfigs.getFirst();
    this.m_shoulderPositionConfig = shoulderConfigs.getSecond();
    this.m_elbowMotionConfig = elbowConfigs.getFirst();
    this.m_elbowPositionConfig = elbowConfigs.getSecond();

    m_currentArmState = ArmState.Stowed;
    m_currentArmDirection = ArmDirection.Up;

    // Set all arm motors to brake
    m_shoulderMasterMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderSlaveMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);

    // Make slave follow master
    m_shoulderSlaveMotor.follow(m_shoulderMasterMotor);

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // Initialize PID
      // m_shoulderMotionConfig.initializeSparkPID(m_shoulderMasterMotor,
      // m_shoulderMasterMotor.getAbsoluteEncoder(), false, false,
      // MOTION_CONFIG_PID_SLOT);
      // m_elbowMotionConfig.initializeSparkPID(m_elbowMotor,
      // m_elbowMotor.getAbsoluteEncoder(), false, false, MOTION_CONFIG_PID_SLOT);

      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getAbsoluteEncoder(),
          false, false, POSITION_CONFIG_PID_SLOT);
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAbsoluteEncoder(), false, false,
          POSITION_CONFIG_PID_SLOT);
    }
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
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
   * Calculate feed forward for shoulder
   * 
   * @return Correctly scaled feed forward based on shoulder angle
   */
  private double calculateShoulderFF() {
    return SHOULDER_FF * Math.sin(Math.toRadians(m_currentArmState.shoulderPosition * CONVERSION_FACTOR));
  }

  /**
   * Calculate feed forward for elbow
   * 
   * @return Correctly scaled feed forward based on elbow angle
   */
  private double calculateElbowFF() {
    return ELBOW_FF * Math.sin(Math.toRadians(m_currentArmState.elbowPosition * CONVERSION_FACTOR));
  }

  /**
   * Check if shoulder motion is complete
   * 
   * @return True if shoulder motion is complete
   */
  private boolean isShoulderMotionComplete() {
    return Math.abs(m_shoulderMasterMotor.getAbsoluteEncoderPosition()
        - m_currentArmState.shoulderPosition) <= Constants.Arm.MOTION_SHOULDER_TOLERANCE;

  }

  /**
   * Check if motion is complete
   * 
   * @return True if elbow motion is complete
   */
  private boolean isElbowMotionComplete() {
    return Math.abs(m_elbowMotor.getAbsoluteEncoderPosition()
        - m_currentArmState.elbowPosition) <= Constants.Arm.MOTION_ELBOW_TOLERANCE;
  }

  private void moveToPositionUp(ArmState armState) {
    if (!isShoulderMotionComplete())
      m_shoulderMasterMotor.set(m_shoulderMotionConfig.calculate(m_shoulderMasterMotor.getAbsoluteEncoderPosition()),
          ControlType.kPosition, calculateShoulderFF(), ArbFFUnits.kPercentOut);
    else if (!isElbowMotionComplete())
      m_elbowMotor.set(m_elbowMotionConfig.calculate(m_elbowMotor.getAbsoluteEncoderPosition()), ControlType.kPosition,
          calculateElbowFF(), ArbFFUnits.kPercentOut);
  }

  private void moveToPositionDown(ArmState armState) {
    if (!isElbowMotionComplete())
      m_elbowMotor.set(m_elbowMotionConfig.calculate(m_elbowMotor.getAbsoluteEncoderPosition()), ControlType.kPosition,
          calculateElbowFF(), ArbFFUnits.kPercentOut);
    else if (!isShoulderMotionComplete())
      m_shoulderMasterMotor.set(m_shoulderMotionConfig.calculate(m_shoulderMasterMotor.getAbsoluteEncoderPosition()),
          ControlType.kPosition, calculateShoulderFF(), ArbFFUnits.kPercentOut);
  }

  public void moveToPosition(ArmState armState, ArmDirection armDirection) {
    Runnable moveToPositions[] = { () -> {
    }, () -> moveToPositionUp(armState), () -> moveToPositionDown(armState) };
    moveToPositions[armDirection.ordinal()].run();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToPosition(m_currentArmState, m_currentArmDirection);
  }

  /**
   * Set desired arm state
   * 
   * @param armState Arm state, which includes shoulder and elbow position
   */
  // m_shoulderMasterMotor.set(m_currentState.shoulderPosition,
  // ControlType.kSmartMotion, 0.0, ArbFFUnits.kPercentOut,
  // MOTION_CONFIG_PID_SLOT);
  // m_elbowMotor.set(m_currentState.elbowPosition, ControlType.kSmartMotion, 0.0,
  // ArbFFUnits.kPercentOut, MOTION_CONFIG_PID_SLOT);
  public void setArmState(ArmState armState) {
    // Update current state
    if (armState != m_currentArmState) {
      m_currentArmDirection = ArmDirection.getArmDirection(m_currentArmState, armState);
    }
    m_currentArmState = armState;

    m_shoulderMotionConfig.setGoal(armState.shoulderPosition);
    m_elbowMotionConfig.setGoal(armState.elbowPosition);
    moveToPosition(m_currentArmState, m_currentArmDirection);

  }

  /**
   * Get current arm state
   * 
   * @return current state
   */
  public ArmState getArmState() {
    return m_currentArmState;
  }

  @Override
  public void close() {
    m_shoulderMasterMotor.close();
    m_elbowMotor.close();
  }
}
