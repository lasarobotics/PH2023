// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ArmSubsystemTest {
  private final double DELTA = 0.1;
  private final boolean MOCK_HARDWARE = false;
  private ArmSubsystem m_armSubsystem;
  private ArmSubsystem.Hardware m_armHardware;

  private SparkMax m_shoulderMasterMotor;
  private SparkMax m_shoulderSlaveMotor;
  private SparkMax m_elbowMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_shoulderMasterMotor = mock(SparkMax.class);
    m_shoulderSlaveMotor = mock(SparkMax.class);
    m_elbowMotor = mock(SparkMax.class);

    // Create Hardware object using mock objects
    m_armHardware = new ArmSubsystem.Hardware(MOCK_HARDWARE, m_shoulderMasterMotor, m_shoulderSlaveMotor, m_elbowMotor);

    // Create ArmSubsystem object
    m_armSubsystem = new ArmSubsystem(m_armHardware,
                                      new Pair<SparkPIDConfig,SparkPIDConfig>(
                                        Constants.Arm.MOTION_SHOULDER_CONFIG,
                                        Constants.Arm.POSITION_SHOULDER_CONFIG
                                      ), 
                                      new Pair<SparkPIDConfig, SparkPIDConfig>(
                                        Constants.Arm.MOTION_ELBOW_CONFIG,
                                        Constants.Arm.POSITION_ELBOW_CONFIG
                                      )
                                    );
  }

  @AfterEach
  public void close() {
    m_armSubsystem.close();
    m_armSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can move arm to stowed position")
  public void stowArm() {
    // Try to move arm to stowed state
    m_armSubsystem.setArmState(ArmState.Stowed);

    // Verify motors are being driven with expected values
    verify(m_shoulderMasterMotor, times(1)).set(AdditionalMatchers.eq(0.89, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
    verify(m_elbowMotor, times(1)).set(AdditionalMatchers.eq(0.57, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                 AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can move arm to ground position")
  public void groundArm() {
    // Try to move arm to ground state
    m_armSubsystem.setArmState(ArmState.Ground);

    // Verify motors are being driven with expected values
    verify(m_shoulderMasterMotor, times(1)).set(AdditionalMatchers.eq(0.85, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
    verify(m_elbowMotor, times(1)).set(AdditionalMatchers.eq(0.29, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion),
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can move arm to middle position")
  public void middleArm() {
    // Try to move arm to middle state
    m_armSubsystem.setArmState(ArmState.Middle);

    // Verify motors are being driven with expected values
    verify(m_shoulderMasterMotor, times(1)).set(AdditionalMatchers.eq(0.63, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
    verify(m_elbowMotor, times(1)).set(AdditionalMatchers.eq(0.28, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion),
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
                                                  
  }
  @Test
  @Order(4)
  @DisplayName("Test if robot can move arm to high position")
  public void highArm() {
    // Try to move arm to high state
    m_armSubsystem.setArmState(ArmState.High);

    // Verify motors are being driven with expected values
    verify(m_shoulderMasterMotor, times(1)).set(AdditionalMatchers.eq(0.60, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));
    verify(m_elbowMotor, times(1)).set(AdditionalMatchers.eq(0.06, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion),
                                                                  AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kPercentOut), ArgumentMatchers.eq(0));                                                  
  }
}
