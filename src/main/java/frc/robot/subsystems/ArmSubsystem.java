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
    Stowed(+0.905, +0.550),
    Ground(+0.840, +0.300),
    Middle(+0.650, +0.280),
    High(+0.550, +0.020);

    public final double shoulderPosition;
    public final double elbowPosition;

    private ArmState(double shoulderPosition, double elbowPosition) {
      this.shoulderPosition = shoulderPosition;
      this.elbowPosition = elbowPosition;
    }
  }

  public enum ArmDirection {
    None, Up, Down;

    public static ArmDirection getArmDirection(ArmState from, ArmState to) {
      int diff = to.ordinal() - from.ordinal();
      return diff < 0 ? Down : diff > 0 ? Up : None;
    }
  }

  private SparkMax m_shoulderMasterMotor;
  private SparkMax m_shoulderSlaveMotor;
  private SparkMax m_elbowMotor;

  private Runnable m_moveToPosition[];

  private ProfiledPIDController m_shoulderMotionConfig;
  private SparkPIDConfig m_shoulderPositionConfig;
  private ProfiledPIDController m_elbowMotionConfig;
  private SparkPIDConfig m_elbowPositionConfig;

  private final double CONVERSION_FACTOR = 360.0;
  private final double SHOULDER_FF = 0.0;
  private final double ELBOW_FF = 0.0;

  private final double TOLERANCE = 0.01;

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
    m_currentArmDirection = ArmDirection.None;

    // Set all arm motors to brake
    m_shoulderMasterMotor.setIdleMode(IdleMode.kCoast);
    m_shoulderSlaveMotor.setIdleMode(IdleMode.kCoast);
    m_elbowMotor.setIdleMode(IdleMode.kCoast);

    // Make slave follow master
    m_shoulderSlaveMotor.follow(m_shoulderMasterMotor);

    // Initialize moveToPosition Runnable Array
    m_moveToPosition = new Runnable[] {
      () -> {},
      () -> armUp(),
      () -> armDown()
    };

    // Set tolerance motion completion
    m_shoulderMotionConfig.setTolerance(TOLERANCE);
    m_elbowMotionConfig.setTolerance(TOLERANCE);

    // Reset motion PID
    m_shoulderMotionConfig.reset(m_shoulderMasterMotor.getAbsoluteEncoderPosition());
    m_elbowMotionConfig.reset(m_elbowMotor.getAbsoluteEncoderPosition());

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // Initialize onboard position PID
      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getAbsoluteEncoder(), false, false);
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAbsoluteEncoder(), false, false);
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
    return SHOULDER_FF
        * Math.sin(Math.toRadians(m_shoulderMasterMotor.getAbsoluteEncoderPosition() * CONVERSION_FACTOR));
  }

  /**
   * Calculate feed forward for elbow
   * 
   * @return Correctly scaled feed forward based on elbow angle
   */
  private double calculateElbowFF() {
    return ELBOW_FF * Math.sin(Math.toRadians(m_elbowMotor.getAbsoluteEncoderPosition() * CONVERSION_FACTOR));
  }

  /**
   * Check if shoulder motion is complete
   * 
   * @return True if shoulder motion is complete
   */
  private boolean isShoulderMotionComplete() {
    return m_shoulderMotionConfig.atGoal();
  }

  /**
   * Check if motion is complete
   * 
   * @return True if elbow motion is complete
   */
  private boolean isElbowMotionComplete() {
    return m_elbowMotionConfig.atGoal();
  }

  /**
   * Move arm position up (first move shoulder up, then elbow)
   */
  private void armUp() {
    if (isShoulderMotionComplete() && isElbowMotionComplete())
      m_currentArmDirection = ArmDirection.None;
    if (!isShoulderMotionComplete())
      m_shoulderMasterMotor.set(m_shoulderMotionConfig.calculate(m_shoulderMasterMotor.getAbsoluteEncoderPosition()),
          ControlType.kPosition, calculateShoulderFF(), ArbFFUnits.kPercentOut);
    if (!isElbowMotionComplete() && isShoulderMotionComplete())
      m_elbowMotor.set(m_elbowMotionConfig.calculate(m_elbowMotor.getAbsoluteEncoderPosition()), ControlType.kPosition,
          calculateElbowFF(), ArbFFUnits.kPercentOut);
  }

  /**
   * Move arm position down (first move elbow, then shoulder)
   */
  private void armDown() {
    if (isShoulderMotionComplete() && isElbowMotionComplete())
      m_currentArmDirection = ArmDirection.None;
    if (!isElbowMotionComplete())
      m_elbowMotor.set(m_elbowMotionConfig.calculate(m_elbowMotor.getAbsoluteEncoderPosition()), ControlType.kPosition,
          calculateElbowFF(), ArbFFUnits.kPercentOut);
    if (!isShoulderMotionComplete() && isElbowMotionComplete())
      m_shoulderMasterMotor.set(m_shoulderMotionConfig.calculate(m_shoulderMasterMotor.getAbsoluteEncoderPosition()),
          ControlType.kPosition, calculateShoulderFF(), ArbFFUnits.kPercentOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_moveToPosition[m_currentArmDirection.ordinal()].run();
  }

  /**
   * Set desired arm state
   * 
   * @param armState Arm state, which includes shoulder and elbow position
   */
  public void setArmState(ArmState armState) {
    // Update current arm direction and state
    m_currentArmDirection = ArmDirection.getArmDirection(m_currentArmState, armState);
    m_currentArmState = armState;

    // Set motion goal for both joints
    m_shoulderMotionConfig.setGoal(armState.shoulderPosition);
    m_elbowMotionConfig.setGoal(armState.elbowPosition);
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
