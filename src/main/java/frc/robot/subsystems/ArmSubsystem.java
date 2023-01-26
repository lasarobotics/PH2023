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

  private SparkMax m_shoulderMotor;
  private SparkMax m_elbowMotor;

  private SparkPIDConfig m_shoulderConfig;
  private SparkPIDConfig m_elbowConfig;

  private final double UPPERARM_LENGTH = 0.9144;
  private final double FOREARM_LENGTH = 0.4445;

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
      // Initialize PID
      m_shoulderConfig.initializeSparkPID(m_shoulderMotor, m_shoulderMotor.getAlternateEncoder());
      m_elbowConfig.initializeSparkPID(m_elbowMotor, m_elbowMotor.getAlternateEncoder());

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

    shoulderAngle = Math.atan(armState.y / armState.x) - Math.atan((FOREARM_LENGTH * elbowAngle) / (UPPERARM_LENGTH + FOREARM_LENGTH * elbowAngle));

    return new Pair<Double, Double>(Math.toDegrees(shoulderAngle), Math.toDegrees(elbowAngle));
  }

  /**
   * Calculate feed forward for arm
   * @param armAngles Angle of upper arm and forearm
   * @return Tuple of shoulder feed forward, elbow feed forward
   */
  private Pair<Double, Double> calculateFF(Pair<Double, Double> armAngles) {
    return new Pair<Double,Double>(
      UPPERARM_LENGTH * Math.cos(armAngles.getFirst()), 
      FOREARM_LENGTH * Math.cos(armAngles.getSecond())
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set desired arm state
   * @param armState Arm state, which includes shoulder and elbow position
   */
  public void setArmState(ArmState armState) {
    // Update current state
    m_currentState = armState;

    // Calculate arm angles
    Pair<Double, Double> armAngles = armIK(armState);

    // Calculate feed forward
    Pair<Double, Double> feedForwards = calculateFF(armAngles);

    // Set shoulder and elbow positions
    m_shoulderMotor.set(armAngles.getFirst(), ControlType.kSmartMotion, feedForwards.getFirst(), ArbFFUnits.kVoltage);
    m_elbowMotor.set(armAngles.getSecond(), ControlType.kSmartMotion, feedForwards.getSecond(), ArbFFUnits.kVoltage);
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
