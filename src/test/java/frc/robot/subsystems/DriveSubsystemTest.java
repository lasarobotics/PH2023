// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import frc.robot.Constants;
import frc.robot.utils.SparkMax;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class DriveSubsystemTest {

  private final double DELTA = 3e-3;
  private final boolean MOCK_HARDWARE = false;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private SparkMax m_lMasterMotor;
  private SparkMax m_rMasterMotor;
  private SparkMax m_leftSlaveMotor;
  private SparkMax m_rightSlaveMotor;
  
  private AHRS m_navx;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_lMasterMotor = mock(SparkMax.class);
    m_rMasterMotor = mock(SparkMax.class);
    m_leftSlaveMotor = mock(SparkMax.class);
    m_rightSlaveMotor = mock(SparkMax.class);
    m_navx = mock(AHRS.class);

    // Create Hardware object using mock objects
    m_drivetrainHardware = new DriveSubsystem.Hardware(MOCK_HARDWARE, m_lMasterMotor, m_rMasterMotor, m_leftSlaveMotor, m_rightSlaveMotor, m_navx);

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(m_drivetrainHardware,
                                          Constants.HID.CONTROLLER_DEADBAND,
                                          Constants.Drive.DRIVE_SLIP_RATIO,
                                          Constants.Drive.DRIVE_kP,
                                          Constants.Drive.DRIVE_kD, 
                                          Constants.Drive.DRIVE_TURN_SCALAR,
                                          Constants.Drive.DRIVE_LOOKAHEAD,
                                          Constants.Drive.DRIVE_TRACTION_CONTROL_CURVE,
                                          Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
                                          Constants.Drive.DRIVE_TURN_INPUT_CURVE);
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can move forward using PID drive")
  public void forward() {
    // Hardcode NAVX sensor return values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)+4.0);

    // Try to drive forward
    m_driveSubsystem.teleopPID(1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can move in reverse using PID drive")
  public void reverse() {
    // Hardcode NAVX sensor return values for angle, and velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)-4.0);

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(-1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can stop using PID drive")
  public void stop() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to stop
    m_driveSubsystem.teleopPID(0.0, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot ignores small throttle input values under threshold")
  public void ignoreSmallThrottleInput() {
    // Hardcode NAVX sensor return values for angle, velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)0.0);

    // Try to drive with small throttle value
    m_driveSubsystem.teleopPID(0.007, 0.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));              
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot ignores small turn input values under threshold")
  public void ignoreSmallTurnInput() {
    // Hardcode NAVX sensor return values for angle, velocityY
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)4.0);

    // Try to drive with small turn value
    m_driveSubsystem.teleopPID(1.0, 0.004);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(8)
  @DisplayName("Test if robot can turn left using PID drive")
  public void turnLeft() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn left
    m_driveSubsystem.teleopPID(0.0, -1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(9)
  @DisplayName("Test if robot can turn right using PID drive")
  public void turnRight() {
    // Hardcode NAVX sensor return value for angle
    when(m_navx.getAngle()).thenReturn(0.0);

    // Try to turn right
    m_driveSubsystem.teleopPID(0.0, 1.0);

    // Verify that left and right motors are being driven with expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(10)
  @DisplayName("Test if robot can limit wheel slip")
  public void tractionControl() {
    // Hardcode NAVX sensor and encoders
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)+0.5);
    when(m_lMasterMotor.getEncoderVelocity()).thenReturn(+2.0);
    when(m_rMasterMotor.getEncoderVelocity()).thenReturn(+2.0);

    // Try to drive at full throttle with traction control enabled
    m_driveSubsystem.enableTractionControl();
    m_driveSubsystem.teleopPID(+1.0, 0.0);

    // Verify that left and right motors are being driven with the expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(+0.135, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(+0.135, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }

  @Test
  @Order(11)
  @DisplayName("Test if robot can disable traction control")
  public void disableTractionControl() {
    // Hardcode NAVX sensor and encoders
    when(m_navx.getAngle()).thenReturn(0.0);
    when(m_navx.getVelocityY()).thenReturn((float)+0.5);
    when(m_lMasterMotor.getEncoderVelocity()).thenReturn(+2.0);
    when(m_rMasterMotor.getEncoderVelocity()).thenReturn(+2.0);

    // Try to drive at full throttle with traction control disabled
    m_driveSubsystem.disableTractionControl();
    m_driveSubsystem.teleopPID(+1.0, 0.0);

    // Verify that left and right motors are being driven with the expected values
    verify(m_lMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
    verify(m_rMasterMotor, times(1)).set(AdditionalMatchers.eq(+1.0, DELTA), ArgumentMatchers.eq(ControlType.kDutyCycle),
                                         AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut));
  }
}
