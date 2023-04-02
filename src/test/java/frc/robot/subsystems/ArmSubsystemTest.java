// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmDirection;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.utils.SparkMax;
import frc.robot.utils.SparkPIDConfig;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ArmSubsystemTest {
  @SuppressWarnings("unused")
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
      new Pair<TrapezoidProfile.Constraints, SparkPIDConfig>(Constants.Arm.MOTION_SHOULDER_CONSTRAINT, Constants.Arm.POSITION_SHOULDER_CONFIG),
      new Pair<TrapezoidProfile.Constraints, SparkPIDConfig>(Constants.Arm.MOTION_ELBOW_CONTRAINT, Constants.Arm.POSITION_ELBOW_CONFIG),
      new Pair<Runnable, Runnable>(() -> {}, () -> {}),
      new Pair<Runnable, Runnable>(() -> {}, () -> {}) 
    );
  }

  @AfterEach
  public void close() {
    m_armSubsystem.close();
    m_armSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can calculate arm direction")
  public void armDirection() {
    // Arm not moving
    assertEquals(ArmDirection.None, ArmDirection.getArmDirection(ArmState.Stowed, ArmState.Stowed));
    assertEquals(ArmDirection.None, ArmDirection.getArmDirection(ArmState.Ground, ArmState.Ground));
    assertEquals(ArmDirection.None, ArmDirection.getArmDirection(ArmState.Middle, ArmState.Middle));
    assertEquals(ArmDirection.None, ArmDirection.getArmDirection(ArmState.High, ArmState.High));

    // Arm moving up
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Stowed, ArmState.Ground));
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Stowed, ArmState.Middle));
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Stowed, ArmState.High));
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Ground, ArmState.Middle));
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Ground, ArmState.High));
    assertEquals(ArmDirection.Up, ArmDirection.getArmDirection(ArmState.Middle, ArmState.High));

    // Arm moving down
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.High, ArmState.Middle));
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.High, ArmState.Ground));
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.High, ArmState.Stowed));
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.Middle, ArmState.Ground));
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.Middle, ArmState.Stowed));
    assertEquals(ArmDirection.Down, ArmDirection.getArmDirection(ArmState.Ground, ArmState.Stowed));
  }
}