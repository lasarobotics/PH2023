// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

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
                    SparkMax telescopeSlaveMotor) {
      this.isHardwareReal = isHardwareReal;
      this.pivotMasterMotor = pivotMasterMotor;
      this.pivotSlaveMotor = pivotSlaveMotor;
      this.telescopeMasterMotor = telescopeMasterMotor;
      this.telescopeSlaveMotor = telescopeSlaveMotor;
    }
  }

  private enum ArmDirection {
    Forward(+1),
    Reverse(-1);

    public final int value;
    private ArmDirection(int value) {
      this.value = value;
    }
  }

  // TODO Set TelescopeState numbers
  private enum TelescopeState {
    Stowed(0.0),
    Ground(0.0),
    Middle(0.0),
    High(0.0);

    public final double value;
    private TelescopeState(double value) {
      this.value = value;
    }
  }

  // TODO Set PivotState numbers
  private enum PivotState {
    Stowed(0.0),
    Ground(0.0),
    Middle(0.0),
    High(0.0);

    public final double value;
    private PivotState(double value) {
      this.value = value;
    }
  }

  public enum ArmState {
    Stowed(PivotState.Stowed, TelescopeState.Stowed),
    Ground(PivotState.Ground, TelescopeState.Ground),
    Middle(PivotState.Middle, TelescopeState.Middle),
    High(PivotState.High, TelescopeState.High);

    public final TelescopeState telescopeState;
    public final PivotState pivotState;
    private ArmState(PivotState pivotState, TelescopeState telescopeState) {
      this.pivotState = pivotState;
      this.telescopeState = telescopeState;
    }
  }

  private SparkMax m_pivotMasterMotor, m_pivotSlaveMotor;
  private SparkMax m_telescopeMasterMotor, m_telescopeSlaveMotor;

  private SparkPIDConfig m_pivotConfig;
  private SparkPIDConfig m_telescopeConfig;

  private ArmDirection m_currentDirection;
  private ArmState m_currentState;

  /**
   * Create an instance of ArmSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param armHardware Hardwave devices required by arm
   * @param pivotConfig PID config for arm pivot
   * @param telescopeConfig PID config for arm telescope
   */
  public ArmSubsystem(Hardware armHardware, SparkPIDConfig pivotConfig, SparkPIDConfig telescopeConfig) {
    this.m_pivotMasterMotor = armHardware.pivotMasterMotor;
    this.m_pivotSlaveMotor = armHardware.pivotSlaveMotor;
    this.m_telescopeMasterMotor = armHardware.telescopeMasterMotor;
    this.m_telescopeSlaveMotor = armHardware.telescopeSlaveMotor;

    m_currentDirection = ArmDirection.Forward;
    m_currentState = ArmState.Stowed;

    // Set all arm motors to brake
    m_pivotMasterMotor.setIdleMode(IdleMode.kBrake);
    m_pivotSlaveMotor.setIdleMode(IdleMode.kBrake);
    m_telescopeMasterMotor.setIdleMode(IdleMode.kBrake);
    m_telescopeSlaveMotor.setIdleMode(IdleMode.kBrake);

    // Set pivot slave motor to follow master
    m_pivotSlaveMotor.follow(m_pivotMasterMotor);

    // Set telescope slave motor to follow master
    m_telescopeSlaveMotor.follow(m_telescopeMasterMotor);

    // Only do this stuff if hardware is real
    if (armHardware.isHardwareReal) {
      // initialize PID
      pivotConfig.initializeSparkPID(m_pivotMasterMotor, m_pivotMasterMotor.getAlternateEncoder());
      telescopeConfig.initializeSparkPID(m_telescopeMasterMotor, m_telescopeMasterMotor.getAnalog(Mode.kAbsolute));
    }
  }

  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware armHardware = new Hardware(isHardwareReal,
                                        new SparkMax(Constants.ArmHardware.ARM_PIVOT_MASTER_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_PIVOT_SLAVE_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_TELESCOPE_MASTER_ID, MotorType.kBrushless),
                                        new SparkMax(Constants.ArmHardware.ARM_TELESCOPE_SLAVE_ID, MotorType.kBrushless));
    return armHardware;
  }

  /**
   * Set arm pivot position
   * @param pivotState Desired pivot state
   */
  private void setPivotState(PivotState pivotState) {
    // Put setpoint within the bounds of the motor
    double setpoint = MathUtil.clamp(pivotState.value, m_pivotConfig.getLowerLimit(), m_pivotConfig.getUpperLimit());

    // Account for arm direction
    setpoint *= m_currentDirection.value;

    // Set pivot motor
    m_pivotMasterMotor.set(setpoint, ControlType.kPosition);
  }

  /**
   * Set arm telescope position
   * @param telescopeState Desired telescope state
   */
  private void setTelescopeState(TelescopeState telescopeState) {
    // Put setpoint within the bounds of the motor
    double setpoint = MathUtil.clamp(telescopeState.value, m_telescopeConfig.getLowerLimit(), m_telescopeConfig.getUpperLimit());

    // Set telescope motor
    m_telescopeMasterMotor.set(setpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set desired arm state
   * @param armState Arm state, which includes pivot and telescope position
   */
  public void setArmState(ArmState armState) {
    // Update current state
    m_currentState = armState;

    // Set telescope and pivot positions
    setPivotState(m_currentState.pivotState);
    setTelescopeState(m_currentState.telescopeState);
  }

  /**
   * Get current arm state
   * @return current state
   */
  public ArmState getArmState() {
    return m_currentState;
  }

  /**
   * Sets direction of arm to forward or reverse
   * @param direction Arm direction
   */
  public void setArmDirection(ArmDirection direction) {
    // Ignore if direction is already correct
    if (direction == m_currentDirection) return;
    
    // If not, then change direction, and set to stowed position
    m_currentDirection = direction;
    setArmState(ArmState.Stowed);
  }

  /**
   * Toggle arm direction
   */
  public void toggleArmDirection() {
    // Toggle direction
    if (m_currentDirection == ArmDirection.Forward) m_currentDirection = ArmDirection.Reverse;
    else m_currentDirection = ArmDirection.Forward;

    // Stow arm in new direction
    setArmState(ArmState.Stowed);
  }

  @Override
  public void close() {
    m_telescopeMasterMotor.close();
    m_telescopeSlaveMotor.close();
    m_pivotMasterMotor.close();
    m_pivotSlaveMotor.close();
  }
}
