// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.MathUtil;

/**
 * Automates the configuration of Spark PID and Smart Motion paramaters
 */
public class SparkPIDConfig {
  private static final double MAX_VOLTAGE = 12.0;
  private static final double MIN_TOLERANCE = 1.0;
  private static final int PID_SLOT = 0;

  private boolean m_smartMotion = false;
  private boolean m_enableSoftLimits = true;

  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_maxRPM = 0.0;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;

  private double m_velocityRPM = 1.0;
  private double m_accelerationRPMPerSec = 1.0;

  private AccelStrategy m_accelStrategy =  AccelStrategy.kTrapezoidal;

   /**
   * Create a SparkPIDConfig, without Smart Motion parameters
   * <p>
   * USE FOR VELOCITY PID ONLY!
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param maxRPM max RPM of encoder
   * @param ticksPerRotation number of ticks in one encoder revolution
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param mechanicalEfficiency mechanical efficiency of mechanism [0.0, +1.0]
   * @param tolerance tolerance of PID loop in ticks per 100ms
   */
  public SparkPIDConfig(boolean sensorPhase, boolean invertMotor, double maxRPM,
                        double kP, double kI, double kD, double mechanicalEfficiency, double tolerance) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_maxRPM = maxRPM * mechanicalEfficiency;
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);

    this.m_enableSoftLimits = false;

    this.m_smartMotion = false;
  }

  /**
   * Create a SparkPIDConfig, with Smart Motion parameters
   * <p>
   * USE FOR POSITION PID ONLY!
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param ticksPerRotation number of ticks in one encoder revolution
   * @param maxRPM max RPM for this motor
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param mechanicalEfficiency mechanical efficiency of mechanism [0.0, +1.0]
   * @param tolerance tolerance of PID loop in ticks
   * @param velocity Smart Motion cruise velocity in RPM
   * @param accelerationRPMPerSec Smart Motion acceleration in RPM
   * @param accelStrategy Smart Motion acceleration strategy
   */
  public SparkPIDConfig(boolean sensorPhase, boolean invertMotor, double maxRPM,
                        double kP, double kI, double kD, double mechanicalEfficiency, double tolerance, 
                        double lowerLimit, double upperLimit, boolean enableSoftLimits,
                        double velocityRPM, double accelerationRPMPerSec, AccelStrategy accelStrategy) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_maxRPM = maxRPM * MathUtil.clamp(mechanicalEfficiency, 0.0, 1.0);
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_tolerance = Math.max(tolerance, MIN_TOLERANCE);
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_enableSoftLimits = enableSoftLimits;
    
    this.m_velocityRPM = velocityRPM;
    this.m_accelerationRPMPerSec = accelerationRPMPerSec;

    this.m_accelStrategy = accelStrategy;

    this.m_smartMotion = true;
  }

  /**
   * Initializes Spark Max PID and Smart Motion parameters
   * @param spark Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeSparkPID(CANSparkMax spark, MotorFeedbackSensor feedbackSensor, 
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch) {
    // Reset Spark to default
    spark.restoreFactoryDefaults();

    // Get PID controller
    SparkMaxPIDController pidController = spark.getPIDController();

    // Configure feedback sensor and set sensor phase
    pidController.setFeedbackDevice(feedbackSensor);
    feedbackSensor.setInverted(m_sensorPhase);
    
    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_upperLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_lowerLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    // Configure forward and reverse limit switches if required
    if (forwardLimitSwitch) {
      spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    }
    if (reverseLimitSwitch) {
      spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }

    // Invert motor if required
    spark.setInverted(m_invertMotor);

    // Configure PID values
    pidController.setP(m_kP, PID_SLOT);
    pidController.setI(m_kI, PID_SLOT);
    pidController.setD(m_kD, PID_SLOT);
    pidController.setOutputRange(-1.0, +1.0);

    pidController.setIZone(m_tolerance * 2, PID_SLOT);

    m_kF = 1 / m_maxRPM;
    pidController.setFF(m_kF, PID_SLOT);

    // Enable voltage compensation
    spark.enableVoltageCompensation(MAX_VOLTAGE);

    // Configure Smart Motion values
    if (m_smartMotion) {  
      pidController.setSmartMotionMaxVelocity(m_velocityRPM, PID_SLOT);
      pidController.setSmartMotionMaxAccel(m_accelerationRPMPerSec, PID_SLOT);
      pidController.setSmartMotionAccelStrategy(m_accelStrategy, PID_SLOT);
    }
  }

  /**
   * Initializes Spark PID and Smart Motion parameters
   * <p>
   * Calls {@link SparkPIDConfig#initializeSparkPID(CANSparkMax, FeedbackDevice, boolean, boolean)} with no limit switches 
   * @param spark Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   */
  public void initializeSparkPID(CANSparkMax spark, MotorFeedbackSensor feedbackSensor) {
    initializeSparkPID(spark, feedbackSensor, false, false);
  }

  /**
   * @return Sensor phase
   */
  public boolean getSensorPhase() {
    return m_sensorPhase;
  }

  /**
   * @return Whether motor is inverted or not
   */
  public boolean getInvertMotor() {
    return m_invertMotor;
  }

  /**
   * @return Proportional gain
   */
  public double getkP() {
    return m_kP;
  }

  /**
   * @return Integral gain
   */
  public double getkI() {
    return m_kI;
  }

  /**
   * @return Derivative gain
   */
  public double getkD() {
    return m_kD;
  }

  /**
   * @return Feed-forward gain
   */
  public double getkF() {
    return m_kF;
  }

  /**
   * @return PID loop tolerance
   */
  public double getTolerance() {
    return m_tolerance;
  }

  /**
   * @return Lower limit of mechanism
   */
  public double getLowerLimit() {
    return m_lowerLimit;
  }

  /**
   * @return Upper limit of mechanism
   */
  public double getUpperLimit() {
    return m_upperLimit;
  }

  /**
   * @return MotionMagic cruise velocity in RPM
   */
  public double getVelocityRPM() {
    return m_velocityRPM;
  }

  /**
   * @return MotionMagic acceleration in RPM per sec
   */
  public double getAccelerationRPMPerSec() {
    return m_accelerationRPMPerSec;
  }
}
