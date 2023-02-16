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
    private SparkMax shoulderMotor, elbowMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax shoulderMotor, 
                    SparkMax elbowMotor) {
      this.isHardwareReal = isHardwareReal;
      this.shoulderMotor = shoulderMotor;
      this.elbowMotor = elbowMotor;
    }
  }

  /**
   * Arm States
   * +X is below shoulder joint
   * +Y is in front of robot
   */
  public enum ArmState {
    Stowed(+0.498, +0.178),
    Ground(+0.914, +0.922),
    Middle(+0.026, +1.322),
    High(-0.249, +1.331);

    public final double x;
    public final double y;
    private ArmState(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  private SparkMax m_shoulderMotor;
  private SparkMax m_elbowMotor;

  private SparkPIDConfig m_shoulderMotionConfig;
  private SparkPIDConfig m_shoulderPositionConfig;
  private SparkPIDConfig m_elbowMotionConfig;
  private SparkPIDConfig m_elbowPositionConfig;

  private boolean shoulderInPositionMode = false;
  private boolean elbowInPositionMode = false;

  private final int MOTION_CONFIG_PID_SLOT = 0;
  private final int POSITION_CONFIG_PID_SLOT = 1;

  private final double UPPERARM_LENGTH = 0.9144;
  private final double FOREARM_LENGTH = 0.4445;

  private final double SHOULDER_FF = 0.0;
  private final double ELBOW_FF = 0.0;

  private Pair<Double, Double> m_armAngles;

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
    this.m_shoulderMotor = armHardware.shoulderMotor;
    this.m_elbowMotor = armHardware.elbowMotor;

    this.m_shoulderMotionConfig = shoulderConfigs.getFirst();
    this.m_shoulderPositionConfig = shoulderConfigs.getSecond();
    this.m_elbowMotionConfig = elbowConfigs.getFirst();
    this.m_elbowPositionConfig = elbowConfigs.getSecond();

    m_currentState = ArmState.Stowed;
    m_armAngles = armIK(ArmState.Stowed);
    
    // Set all arm motors to brake
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // Initialize PID
      m_shoulderMotionConfig.initializeSparkPID(m_shoulderMotor, m_shoulderMotor.getAlternateEncoder(), false, false, MOTION_CONFIG_PID_SLOT);
      m_elbowMotionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAlternateEncoder(), false, false, MOTION_CONFIG_PID_SLOT);

      m_shoulderPositionConfig.initializeSparkPID(m_shoulderMotor, m_shoulderMotor.getAlternateEncoder(), false, false, POSITION_CONFIG_PID_SLOT);
      m_elbowPositionConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAlternateEncoder(), false, false, POSITION_CONFIG_PID_SLOT);

      // Set conversion factor
      double conversionFactor = 360;
      m_shoulderMotor.getAlternateEncoder().setPositionConversionFactor(conversionFactor);
      m_elbowMotor.getAlternateEncoder().setPositionConversionFactor(conversionFactor);
    }
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(isHardwareReal,
                                        new SparkMax(Constants.ArmHardware.ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_ELBOW_MOTOR_ID, MotorType.kBrushless));
    return armHardware;
  }

  /**
   * Calculate arm joint angles given end effector coordinate
   * @param x horizontal coordinate of end effector relative to shoulder joint
   * @param y vertical coordinate of end effector relative to shoulder joint
   * @return Tuple of shoulder angle, elbow angle in degrees
   */
  private Pair<Double, Double> armIK(ArmState armState) {
    double shoulderAngle = 0.0;
    double elbowAngle = 0.0;

    elbowAngle = Math.acos(
      (Math.pow(armState.x, 2) + Math.pow(armState.y, 2) - Math.pow(UPPERARM_LENGTH, 2) - Math.pow(FOREARM_LENGTH, 2))
      / 
      (2 * UPPERARM_LENGTH * FOREARM_LENGTH)
    );

    shoulderAngle = Math.atan(armState.y / armState.x) - Math.atan((FOREARM_LENGTH * Math.sin(elbowAngle)) / (UPPERARM_LENGTH + FOREARM_LENGTH * Math.cos(elbowAngle)));
    if (armState.x < 0) shoulderAngle += Math.PI;

    return new Pair<Double, Double>(Math.toDegrees(shoulderAngle), Math.toDegrees(elbowAngle));
  }

  /**
   * Calculate feed forward for arm
   * @param armAngles Angle of shoulder and elbow in degrees
   * @return Tuple of shoulder feed forward, elbow feed forward
   */
  private Pair<Double, Double> calculateFF(Pair<Double, Double> armAngles) {
    return new Pair<Double,Double>(
      SHOULDER_FF * Math.cos(Math.toRadians(90 - armAngles.getFirst())), 
      ELBOW_FF * Math.cos(Math.toRadians(90 - armAngles.getSecond()))
    );
  }

  
  public boolean isShoulderMotionComplete() {
    return Math.abs(m_shoulderMotor.get()) >= Arm.MOTOR_END_VELOCITY_THRESHOLD && m_shoulderMotor.getAlternateEncoderPosition() == m_armAngles.getFirst();
  }


  public boolean isElbowMotionComplete() {
    return Math.abs(m_shoulderMotor.get()) >= Arm.MOTOR_END_VELOCITY_THRESHOLD && m_shoulderMotor.getAlternateEncoderPosition() == m_armAngles.getSecond();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.shoulderInPositionMode = this.isShoulderMotionComplete();
    this.elbowInPositionMode = this.isElbowMotionComplete();
  }

  /**
   * Set desired arm state
   * @param armState Arm state, which includes shoulder and elbow position
   */
  public void setArmState(ArmState armState) {
    // Update current state
    m_currentState = armState;

    // Calculate arm angles
    m_armAngles = armIK(armState);

    // Calculate feed forward
    Pair<Double, Double> feedForwards = calculateFF(m_armAngles);

    // Set shoulder and elbow positions
    if(this.shoulderInPositionMode)
      m_shoulderMotor.set(m_armAngles.getFirst(), ControlType.kSmartMotion, feedForwards.getFirst(), ArbFFUnits.kVoltage, POSITION_CONFIG_PID_SLOT);
    else
      m_shoulderMotor.set(m_armAngles.getFirst(), ControlType.kSmartMotion, feedForwards.getFirst(), ArbFFUnits.kVoltage, MOTION_CONFIG_PID_SLOT);
    if(this.elbowInPositionMode)
      m_elbowMotor.set(m_armAngles.getSecond(), ControlType.kSmartMotion, feedForwards.getSecond(), ArbFFUnits.kVoltage, POSITION_CONFIG_PID_SLOT);
    else
      m_elbowMotor.set(m_armAngles.getSecond(), ControlType.kSmartMotion, feedForwards.getSecond(), ArbFFUnits.kVoltage, MOTION_CONFIG_PID_SLOT);
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
