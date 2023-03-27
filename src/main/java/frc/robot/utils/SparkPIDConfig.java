// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

/**
 * Automates the configuration of Spark PID and Smart Motion paramaters
 */
public class SparkPIDConfig {
  private static final double MAX_VOLTAGE = 12.0;
  private static final int PID_SLOT = 0;
  
  private boolean m_enableSoftLimits = true;

  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;

  /**
   * Create a SparkPIDConfig, without Smart Motion parameters
   * <p>
   * USE FOR VELOCITY PID ONLY!
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feed-forward gain
   * @param tolerance tolerance of PID loop in ticks per 100ms
   */
  public SparkPIDConfig(boolean sensorPhase, boolean invertMotor,
                        double kP, double kI, double kD, double kF, double tolerance) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_kF = kF;
    this.m_tolerance = tolerance;

    this.m_enableSoftLimits = false;
  }

  /**
   * Create a SparkPIDConfig, with Smart Motion parameters
   * <p>
   * USE FOR POSITION PID ONLY!
   * @param sensorPhase set sensor phase of encoder
   * @param invertMotor invert motor or not
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kF feed-forward gain
   * @param tolerance tolerance of PID loop in ticks
   * @param lowerLimit lower soft limit
   * @param upperLimit upper soft limit
   * @param enableSoftLimits true to enable soft limits
   * @param velocity Smart Motion cruise velocity in RPM
   * @param accelerationRPMPerSec Smart Motion acceleration in RPM per second
   * @param accelStrategy Smart Motion acceleration strategy
   */
  public SparkPIDConfig(boolean sensorPhase, boolean invertMotor,
                        double kP, double kI, double kD, double kF, double tolerance, 
                        double lowerLimit, double upperLimit, boolean enableSoftLimits) {
    this.m_sensorPhase = sensorPhase;
    this.m_invertMotor = invertMotor;
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_kF = kF;
    this.m_tolerance = tolerance;
    this.m_lowerLimit = lowerLimit;
    this.m_upperLimit = upperLimit;
    this.m_enableSoftLimits = enableSoftLimits;
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
    try {
      pidController.setFeedbackDevice(feedbackSensor);
      feedbackSensor.setInverted(m_sensorPhase);
    } catch (IllegalArgumentException e) {}
    
    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_upperLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_lowerLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    // Configure forward and reverse limit switches if required, and disable soft limit
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

    // Don't wrap PID positions
    pidController.setPositionPIDWrappingEnabled(false);

    // Configure PID values
    pidController.setP(m_kP, PID_SLOT);
    pidController.setI(m_kI, PID_SLOT);
    pidController.setD(m_kD, PID_SLOT);
    pidController.setFF(m_kF, PID_SLOT);
    pidController.setOutputRange(-1.0, +1.0);
    pidController.setIZone(m_tolerance * 2, PID_SLOT);

    // Enable voltage compensation
    spark.enableVoltageCompensation(MAX_VOLTAGE);

    // Write settings to onboard flash
    for (int i = 0; i < 10; i++) spark.burnFlash();
  }


  /**
   * Initializes Spark Max PID and Smart Motion parameters
   * @param spark Spark motor controller to apply settings to
   * @param feedbackSensor Feedback device to use for Spark PID
   * @param forwardLimitSwitch Enable forward limit switch
   * @param reverseLimitSwitch Enable reverse limit switch
   */
  public void initializeSparkPID(CANSparkMax spark, MotorFeedbackSensor feedbackSensor, 
                                 boolean forwardLimitSwitch, boolean reverseLimitSwitch, int PID_SLOT) {
    // Reset Spark to default
    spark.restoreFactoryDefaults();

    // Get PID controller
    SparkMaxPIDController pidController = spark.getPIDController();

    // Configure feedback sensor and set sensor phase
    try {
      pidController.setFeedbackDevice(feedbackSensor);
      feedbackSensor.setInverted(m_sensorPhase);
    } catch (IllegalArgumentException e) {}
    
    // Configure forward and reverse soft limits
    if (m_enableSoftLimits) {
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) m_upperLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) m_lowerLimit);
      spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    // Configure forward and reverse limit switches if required, and disable soft limit
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
    pidController.setFF(m_kF, PID_SLOT);
    pidController.setOutputRange(-1.0, +1.0);
    pidController.setIZone(m_tolerance * 2, PID_SLOT);

    // Enable voltage compensation
    spark.enableVoltageCompensation(MAX_VOLTAGE);

    // Write settings to onboard flash
    for (int i = 0; i < 10; i++) spark.burnFlash();
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
}
