// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax pivotMasterMotor, pivotSlaveMotor;
    private SparkMax telescopeMasterMotor, telescopeSlaveMotor;

    public Hardware(boolean isHardwareReal,
                    SparkMax pivotMasterMotor, 
                    SparkMax pivotSlaveMotor, 
                    SparkMax telescopeMasterMotor,
                    SparkMax telescopeSlaveMotor
                    ) {
      this.isHardwareReal = isHardwareReal;
      this.pivotMasterMotor = pivotMasterMotor;
      this.pivotSlaveMotor = pivotSlaveMotor;
      this.telescopeMasterMotor = telescopeMasterMotor;
      this.telescopeSlaveMotor = telescopeSlaveMotor;
    }
  }

  public enum ArmState {
    Top(Constants.Arm.TELESCOPE_CONFIG.getUpperLimit(), Constants.Arm.PIVOT_CONFIG.getUpperLimit()),
    Bottom(Constants.Arm.TELESCOPE_CONFIG.getLowerLimit(), Constants.Arm.PIVOT_CONFIG.getLowerLimit());

    public final double telescopePosition;
    public final double pivotPosition;
    private ArmState(double telescopePosition, double pivotPosition) {
      this.telescopePosition = telescopePosition;
      this.pivotPosition = pivotPosition;
    }
  }

  private SparkMax m_pivotMasterMotor, m_pivotSlaveMotor;
  private SparkMax m_telescopeMasterMotor, m_telescopeSlaveMotor;

  private SparkPIDConfig m_pivotConfig;
  private SparkPIDConfig m_telescopeConfig;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(Hardware armHardware, SparkPIDConfig pivotConfig, SparkPIDConfig telescopeConfig) {
    this.m_pivotMasterMotor = armHardware.pivotMasterMotor;
    this.m_pivotSlaveMotor = armHardware.pivotSlaveMotor;
    this.m_telescopeMasterMotor = armHardware.telescopeMasterMotor;
    this.m_telescopeSlaveMotor = armHardware.telescopeSlaveMotor;

    // Set all drive motors to brake
    m_pivotMasterMotor.setIdleMode(IdleMode.kBrake);
    m_pivotSlaveMotor.setIdleMode(IdleMode.kBrake);
    m_telescopeMasterMotor.setIdleMode(IdleMode.kBrake);
    m_telescopeSlaveMotor.setIdleMode(IdleMode.kBrake);

    // Set telescope slave motor to follow master
    m_telescopeSlaveMotor.follow(m_telescopeMasterMotor);

    // Set pivot slave motor to follow master
    m_pivotSlaveMotor.follow(m_pivotMasterMotor);

    // initialize PID
    pivotConfig.initializeSparkPID(m_pivotMasterMotor, m_pivotMasterMotor.getEncoder());
    telescopeConfig.initializeSparkPID(m_telescopeMasterMotor, m_telescopeMasterMotor.getEncoder());
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(isHardwareReal,
                                               new SparkMax(Constants.ArmHardware.ARM_PIVOT_MASTER_ID, MotorType.kBrushless),
                                               new SparkMax(Constants.ArmHardware.ARM_PIVOT_SLAVE_ID, MotorType.kBrushless),
                                               new SparkMax(Constants.ArmHardware.ARM_TELESCOPE_MASTER_ID, MotorType.kBrushless),
                                               new SparkMax(Constants.ArmHardware.ARM_TELESCOPE_SLAVE_ID, MotorType.kBrushless));
    return armHardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPivotPosition(double setpoint) {
    // Put setpoint within the bounds of the motor
    setpoint = MathUtil.clamp(setpoint, m_pivotConfig.getLowerLimit(), m_pivotConfig.getUpperLimit());

    // Set pivot motor
    m_pivotMasterMotor.set(setpoint, ControlType.kPosition);
  }

  public void setTelescopePosition(double setpoint) {
    // Put setpoint within the bounds of the motor
    setpoint = MathUtil.clamp(setpoint, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());

    // Set telescope motor
    m_telescopeMasterMotor.set(setpoint, ControlType.kPosition);
  }

  public void setArmState(ArmState armState) {
    // Set telescope and pivot positions from enum
    setPivotPosition(armState.pivotPosition);
    setTelescopePosition(armState.telescopePosition);
  }

  @Override
  public void close() {
    m_telescopeMasterMotor.close();
    m_telescopeSlaveMotor.close();
    m_pivotMasterMotor.close();
    m_pivotSlaveMotor.close();
  }

}
