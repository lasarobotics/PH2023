// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Stowed(+0.0, +0.0),
    Ground(+0.0, -55.4),
    Middle(-126.5, -81.5),
    High(-120.0, -61.0);

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

  private final double SHOULDER_STRAIGHT_POSITION = 0.665;
  private final double ELBOW_STRAIGHT_POSITION = 0.552;
  private final double SHOULDER_THRESHOLD = 0.35;

  private final double CONVERSION_FACTOR = 360.0;
  private final double SHOULDER_FF = 0.04;
  private final double ELBOW_FF = 0.00;
  private final double GROUND_ARM_FF = -0.8;
  private final double STOW_ARM_FF = +0.8;

  private final double MANUAL_CONTROL_SCALAR = 0.1;

  private final double AUTO_SHOULDER_POSITION = 0.758;
  private final double AUTO_ELBOW_POSITION = 0.670;

  private ArmState m_currentArmState;
  private ArmState m_prevArmState;
  private ArmDirection m_currentArmDirection;

  private TrapezoidProfile m_shoulderMotionProfile;
  private TrapezoidProfile m_elbowMotionProfile;

  private Timer m_shoulderMotionTimer;
  private Timer m_elbowMotionTimer;

  private Runnable m_enableTurnRateLimit;
  private Runnable m_disableTurnRateLimit;

  private Runnable m_enableDriveSlow;
  private Runnable m_disableDriveSlow;

  private boolean m_enableManualControl = false;

  /**
   * Create an instance of ArmSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param armHardware     Hardwave devices required by arm
   * @param shoulderConfigs Paired PID config for arm shoulder
   * @param elbowConfigs    Paired PID config for arm elbow
   * @param turnLimit       DriveSubsystem methods to enable/diable turn rate
   *                        limiter (enable, disable)
   */
  public ArmSubsystem(Hardware armHardware, Pair<TrapezoidProfile.Constraints, SparkPIDConfig> shoulderConfigs,
      Pair<TrapezoidProfile.Constraints, SparkPIDConfig> elbowConfigs, Pair<Runnable, Runnable> turnLimit, Pair<Runnable, Runnable> moveLimit) {
    this.m_shoulderMasterMotor = armHardware.shoulderMasterMotor;
    this.m_shoulderSlaveMotor = armHardware.shoulderSlaveMotor;
    this.m_elbowMotor = armHardware.elbowMotor;

    this.m_shoulderMotionConstraint = shoulderConfigs.getFirst();
    this.m_shoulderPositionConfig = shoulderConfigs.getSecond();

    this.m_elbowMotionConstraint = elbowConfigs.getFirst();
    this.m_elbowPositionConfig = elbowConfigs.getSecond();

    this.m_enableTurnRateLimit = turnLimit.getFirst();
    this.m_disableTurnRateLimit = turnLimit.getSecond();

    this.m_enableDriveSlow = moveLimit.getFirst();
    this.m_disableDriveSlow = moveLimit.getSecond();

    // Initialize arm state and direction
    m_currentArmState = ArmState.Stowed;
    m_currentArmDirection = ArmDirection.None;
    m_prevArmState = m_currentArmState;

    // Initialize timers
    m_shoulderMotionTimer = new Timer();
    m_elbowMotionTimer = new Timer();

    // Set all arm motors to brake
    m_shoulderMasterMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderSlaveMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);

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
      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMasterMotor, m_shoulderMasterMotor.getEncoder());
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getEncoder());

      // Reset NEO encoder
      m_shoulderMasterMotor.resetEncoder();
      m_elbowMotor.resetEncoder();
    }
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(
      isHardwareReal,
      new SparkMax(Constants.ArmHardware.ARM_SHOULDER_MASTER_MOTOR_ID, MotorType.kBrushless),
      new SparkMax(Constants.ArmHardware.ARM_SHOULDER_SLAVE_MOTOR_ID, MotorType.kBrushless),
      new SparkMax(Constants.ArmHardware.ARM_ELBOW_MOTOR_ID, MotorType.kBrushless)
    );
    return armHardware;
  }

  /**
   * Calculate feed forward for shoulder
   * 
   * @return Correctly scaled feed forward based on shoulder angle
   */
  private double calculateShoulderFF() {
    return SHOULDER_FF * Math.cos(Math.toRadians((m_shoulderMasterMotor.getAbsoluteEncoderPosition() - SHOULDER_STRAIGHT_POSITION) * CONVERSION_FACTOR));
  }

  /**
   * Calculate feed forward for elbow
   * 
   * @return Correctly scaled feed forward based on elbow angle
   */
  private double calculateElbowFF() {
    if (m_currentArmState == ArmState.Ground && !isElbowMotionComplete()) return GROUND_ARM_FF;
    if (m_prevArmState == ArmState.Ground && m_currentArmState == ArmState.Stowed && !isElbowMotionComplete()) return STOW_ARM_FF;

    return ELBOW_FF * Math.cos(Math.toRadians((m_elbowMotor.getAbsoluteEncoderPosition() - ELBOW_STRAIGHT_POSITION) * CONVERSION_FACTOR));
  }

  /**
   * Check if shoulder motion is complete
   * 
   * @return True if shoulder motion is complete
   */
  public boolean isShoulderMotionComplete() {
    return m_shoulderMotionProfile.isFinished(m_shoulderMotionTimer.get());
  }

  /**
   * Check if motion is complete
   * 
   * @return True if elbow motion is complete
   */
  public boolean isElbowMotionComplete() {
    return m_elbowMotionProfile.isFinished(m_elbowMotionTimer.get());
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
        m_shoulderMotionProfile.calculate(m_shoulderMotionTimer.get()).position,
        ControlType.kPosition,
        calculateShoulderFF(),
        ArbFFUnits.kPercentOut
      );

      // Advance elbow start time
      if (Math.abs(m_shoulderMasterMotor.getEncoderPosition() - m_currentArmState.shoulderPosition) < SHOULDER_THRESHOLD)
        m_elbowMotionTimer.restart();
    }

    // Move elbow once shoulder is past threshold
    if (!isElbowMotionComplete() &&
        Math.abs(m_shoulderMasterMotor.getEncoderPosition() - m_currentArmState.shoulderPosition) > SHOULDER_THRESHOLD) {
      // Set elbow position
      m_elbowMotor.set(
        m_elbowMotionProfile.calculate(m_elbowMotionTimer.get()).position,
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
    // Move elbow
    m_elbowMotor.set(
      m_elbowMotionProfile.calculate(m_elbowMotionTimer.get()).position,
      ControlType.kPosition,
      calculateElbowFF(),
      ArbFFUnits.kPercentOut
    );

    // Move shoulder
    m_shoulderMasterMotor.set(
      m_shoulderMotionProfile.calculate(m_shoulderMotionTimer.get()).position,
      ControlType.kPosition,
      calculateShoulderFF(),
      ArbFFUnits.kPercentOut
    );

    // Hold arm position if complete
    armHoldIfComplete();
  }

  /**
   * Set desired arm state
   * 
   * @param armState Arm state, which includes shoulder and elbow position
   */
  private void setArmState(double shoulderPosition, double elbowPosition) {
    // Generate states
    TrapezoidProfile.State desiredShoulderState = new TrapezoidProfile.State(shoulderPosition, 0.0);
    TrapezoidProfile.State currentShoulderState = new TrapezoidProfile.State(m_shoulderMasterMotor.getEncoderPosition(), m_shoulderMasterMotor.getEncoderVelocity() / 60);
    TrapezoidProfile.State desiredElbowState = new TrapezoidProfile.State(elbowPosition, 0.0);
    TrapezoidProfile.State currentElbowState = new TrapezoidProfile.State(m_elbowMotor.getEncoderPosition(), m_elbowMotor.getEncoderVelocity() / 60);

    // Restart motion timers
    m_shoulderMotionTimer.restart();
    m_elbowMotionTimer.restart();

    // Generate motion profile for both joints
    m_shoulderMotionProfile = new TrapezoidProfile(m_shoulderMotionConstraint, desiredShoulderState, currentShoulderState);
    m_elbowMotionProfile = new TrapezoidProfile(m_elbowMotionConstraint, desiredElbowState, currentElbowState);
  }

  /**
   * Set arm to auto position
   */
  public void setAutoPosition() {
   setArmState(AUTO_SHOULDER_POSITION, AUTO_ELBOW_POSITION);
  }

  /**
   * Enable manual motor control
   */
  public void toggleManualControl() {
    m_enableManualControl = !m_enableManualControl;
  }

  /**
   * Manually control elbow motor
   * 
   * @param elbowRequest
   */
  public void manualElbowRequest(double elbowRequest) {
    if (m_enableManualControl) m_elbowMotor.set(elbowRequest * MANUAL_CONTROL_SCALAR, ControlType.kDutyCycle);
  }

  /**
   * Manually control shoulder motor
   * 
   * @param shoulderRequest
   */
  public void manualShoulderRequest(double shoulderRequest) {
    if (m_enableManualControl) m_shoulderMasterMotor.set(shoulderRequest * MANUAL_CONTROL_SCALAR, ControlType.kDutyCycle);
  }

  /**
   * Hold current position
   */
  public void manualHoldPosition() {
    m_shoulderMasterMotor.set(
      m_shoulderMasterMotor.getAbsoluteEncoderPosition(),
      ControlType.kPosition,
      calculateShoulderFF(),
      ArbFFUnits.kPercentOut
    );
    m_elbowMotor.set(
      m_elbowMotor.getAbsoluteEncoderPosition(),
      ControlType.kPosition,
      calculateElbowFF(),
      ArbFFUnits.kPercentOut
    );
  }

  @Override
  public void periodic() {
    smartDashboard();
    if (!m_enableManualControl) m_moveToPosition[m_currentArmDirection.ordinal()].run();
  }
  
  public void smartDashboard() {
    // System.out.println("SP: " + m_shoulderMasterMotor.getAbsoluteEncoderPosition() +  " EP: " + m_elbowMotor.getAbsoluteEncoderPosition() );
    SmartDashboard.putBoolean("Arm Manual", m_enableManualControl);
  }

  /**
   * Set desired arm state
   * 
   * @param armState Arm state, which includes shoulder and elbow position
   */
  public void setArmState(ArmState armState) {
    // Update current arm state
    m_currentArmDirection = ArmDirection.getArmDirection(m_currentArmState, armState);
    m_prevArmState = m_currentArmState;
    m_currentArmState = armState;

    // Limit turn rate if arm is not stowed
    if (!armState.equals(ArmState.Stowed)) m_enableTurnRateLimit.run();
    else m_disableTurnRateLimit.run();
    
    // Limit drive speed if arm is at high or middle position
    if (armState.equals(ArmState.High) || armState.equals(ArmState.Middle)) m_enableDriveSlow.run();
    else m_disableDriveSlow.run();

    // Set arm state
    setArmState(m_currentArmState.shoulderPosition, m_currentArmState.elbowPosition);
  }

  /**
   * Get current arm state
   * 
   * @return current state
   */
  public ArmState getArmState() {
    return m_currentArmState;
  }

  /**
   * Stop elbow motor
   */
  public void elbowStop() {
    m_elbowMotor.stopMotor();
  }

  /**
   * Stop shoulder motor
   */
  public void shoulderStop() {
    m_shoulderMasterMotor.stopMotor();
  }

  @Override
  public void close() {
    m_shoulderMasterMotor.close();
    m_elbowMotor.close();
  }
}
