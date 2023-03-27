// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    Ground(+0.846, +0.589),
    Middle(+0.650, +0.280),
    High(+0.550, +0.200);

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

  private TrapezoidProfile.Constraints m_shoulderMotionConstraint;
  private SparkPIDConfig m_shoulderPositionConfig;
  private TrapezoidProfile.Constraints m_elbowMotionConstraint;
  private SparkPIDConfig m_elbowPositionConfig;

  private final double SHOULDER_THRESHOLD = 0.7;
  private final double CONVERSION_FACTOR = 360.0;
  private final double SHOULDER_FF = 0.01;
  private final double ELBOW_FF = 0.0;

  private ArmState m_currentArmState;
  private ArmDirection m_currentArmDirection;

  private TrapezoidProfile m_shoulderMotionProfile;
  private TrapezoidProfile m_elbowMotionProfile;

  private Instant m_shoulderStartTime;
  private Instant m_elbowStartTime;

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

  public ArmSubsystem(Hardware armHardware, Pair<TrapezoidProfile.Constraints, SparkPIDConfig> shoulderConfigs,
      Pair<TrapezoidProfile.Constraints, SparkPIDConfig> elbowConfigs) {
    this.m_shoulderMasterMotor = armHardware.shoulderMasterMotor;
    this.m_shoulderSlaveMotor = armHardware.shoulderSlaveMotor;
    this.m_elbowMotor = armHardware.elbowMotor;

    this.m_shoulderMotionConstraint = shoulderConfigs.getFirst();
    this.m_shoulderPositionConfig = shoulderConfigs.getSecond();
    this.m_elbowMotionConstraint = elbowConfigs.getFirst();
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

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // Initialize onboard position PID
      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getAbsoluteEncoder());
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAbsoluteEncoder());
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
  public boolean isShoulderMotionComplete() {
    return m_shoulderMotionProfile.isFinished(Duration.between(m_shoulderStartTime, Instant.now()).getSeconds());
  }

  /**
   * Check if motion is complete
   * 
   * @return True if elbow motion is complete
   */
  public boolean isElbowMotionComplete() {
    return m_elbowMotionProfile.isFinished(Duration.between(m_elbowStartTime, Instant.now()).getSeconds());
  }

  /**
   * Hold position of arm when both joint motions are complete
   */
  private void armHoldIfComplete() {
    // Hold position when both joints are complete
    if (isShoulderMotionComplete() && isElbowMotionComplete()) {
      m_shoulderMasterMotor.set(m_currentArmState.shoulderPosition, ControlType.kPosition, calculateShoulderFF(), ArbFFUnits.kPercentOut);
      m_elbowMotor.set(m_currentArmState.elbowPosition, ControlType.kPosition, calculateElbowFF(), ArbFFUnits.kPercentOut);
      m_currentArmDirection = ArmDirection.None;
    }
  }

  /**
   * Move arm position up (first move shoulder up, then elbow)
   */
  private void armUp() {
    // Move shoulder first
    if (!isShoulderMotionComplete()) {
      // Set shoulder position
      m_shoulderMasterMotor.set(
        m_shoulderMotionProfile.calculate(Duration.between(m_shoulderStartTime, Instant.now()).getSeconds()).position,
        ControlType.kPosition,
        calculateShoulderFF(),
        ArbFFUnits.kPercentOut
      );

      // Advance elbow start time
      if (m_shoulderMasterMotor.getAbsoluteEncoderPosition() > SHOULDER_THRESHOLD) m_elbowStartTime = Instant.now();
    }

    // Move elbow once shoulder is past threshold
    if (m_shoulderMasterMotor.getAbsoluteEncoderPosition() < SHOULDER_THRESHOLD) {
      // Set elbow position
      m_elbowMotor.set(
        m_elbowMotionProfile.calculate(Duration.between(m_elbowStartTime, Instant.now()).getSeconds()).position,
        ControlType.kPosition,
        calculateElbowFF(),
        ArbFFUnits.kPercentOut
      );
    }

    // Hold arm position if complete
    armHoldIfComplete();
  }

  /**
   * Move arm position down (first move elbow, then shoulder)
   */
  private void armDown() {
    // Set elbow position
    m_elbowMotor.set(
      m_elbowMotionProfile.calculate(Duration.between(m_elbowStartTime, Instant.now()).getSeconds()).position,
      ControlType.kPosition,
      calculateElbowFF(),
      ArbFFUnits.kPercentOut
    );

    // Set shoulder position
    m_shoulderMasterMotor.set(
      m_shoulderMotionProfile.calculate(Duration.between(m_shoulderStartTime, Instant.now()).getSeconds()).position,
      ControlType.kPosition,
      calculateShoulderFF(),
      ArbFFUnits.kPercentOut
    );

    // Hold arm position if complete
    armHoldIfComplete();
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

    // Generate states
    TrapezoidProfile.State desiredShoulderState = new TrapezoidProfile.State(armState.shoulderPosition, 0.0);
    TrapezoidProfile.State currentShoulderState = new TrapezoidProfile.State(m_shoulderMasterMotor.getAbsoluteEncoderPosition(), m_shoulderMasterMotor.getAbsoluteEncoderVelocity() / 60);
    TrapezoidProfile.State desiredElbowState = new TrapezoidProfile.State(armState.elbowPosition, 0.0);
    TrapezoidProfile.State currentElbowState = new TrapezoidProfile.State(m_elbowMotor.getAbsoluteEncoderPosition(), m_elbowMotor.getAbsoluteEncoderVelocity() / 60);

    // Generate motion profile for both joints
    m_shoulderStartTime = Instant.now();
    m_elbowStartTime = Instant.now();
    m_shoulderMotionProfile = new TrapezoidProfile(m_shoulderMotionConstraint, desiredShoulderState, currentShoulderState);
    m_elbowMotionProfile = new TrapezoidProfile(m_elbowMotionConstraint, desiredElbowState, currentElbowState);
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
