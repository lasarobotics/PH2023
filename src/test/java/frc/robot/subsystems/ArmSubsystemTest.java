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

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.utils.SparkMax;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ArmSubsystemTest {
  private final double DELTA = 2e-3;
  private final boolean MOCK_HARDWARE = false;
  private ArmSubsystem m_armSubsystem;
  private ArmSubsystem.Hardware m_armHardware;

  private SparkMax m_shoulderMotor;
  private SparkMax m_elbowMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_shoulderMotor = mock(SparkMax.class);
    m_elbowMotor = mock(SparkMax.class);

    // Create Hardware object using mock objects
    m_armHardware = new ArmSubsystem.Hardware(MOCK_HARDWARE, m_shoulderMotor, m_elbowMotor);

    // Create ArmSubsystem object
    m_armSubsystem = new ArmSubsystem(m_armHardware, Constants.Arm.SHOULDER_CONFIG, Constants.Arm.ELBOW_CONFIG);
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
    verify(m_shoulderMotor, times(1)).set(AdditionalMatchers.eq(1.364, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                    AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
    verify(m_elbowMotor, times(1)).set(AdditionalMatchers.eq(158.058, DELTA), ArgumentMatchers.eq(ControlType.kSmartMotion), 
                                                                 AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ArbFFUnits.kVoltage));
  }
}
