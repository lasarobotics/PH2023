// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.utils.PIDConstants;
import frc.robot.utils.SparkPIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Global {
    public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;

    // Motor RPMs, encoder values, and gear ratios
    public static final int NEO_MAX_RPM = 5880;
    public static final int NEO_ENCODER_TICKS_PER_ROTATION = 42;
    public static final int REV_ENCODER_TICKS_PER_ROTATION = 8192;
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.10;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(0.019, 0.0, 0.0012, 0.0);
    public static final PIDConstants DRIVE_BALANCE_PID = new PIDConstants(0.007, 0.0, 0.00002, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.15;
    public static final double DEFAULT_DRIVE_TURN_SCALAR = 35.0;
    public static final double DRIVE_LOOKAHEAD = 3;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.5, 1.0 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 1.975, 3.95 };
    private static final double DRIVE_TRACTION_CONTROL_CURVE_X[] = { 0.0, 1.975, 3.95 };
    private static final double DRIVE_TRACTION_CONTROL_CURVE_Y[] = { 0.0, 0.5, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TRACTION_CONTROL_CURVE = SPLINE_INTERPOLATOR
        .interpolate(DRIVE_TRACTION_CONTROL_CURVE_X, DRIVE_TRACTION_CONTROL_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  }

  public static class Arm {
    // Arm shoulder motion settings
    private static final double MOTION_SHOULDER_VELOCITY = 0.4;
    private static final double MOTION_SHOULDER_ACCELERATION = 0.5;
    public static final Constraints MOTION_SHOULDER_CONSTRAINT = new TrapezoidProfile.Constraints(MOTION_SHOULDER_VELOCITY, MOTION_SHOULDER_ACCELERATION);
    
    // Arm shoulder position PID settings
    private static final double POSITION_SHOULDER_kP = 4.5;
    private static final double POSITION_SHOULDER_kI = 0.0;
    private static final double POSITION_SHOULDER_kD = 15.0;
    private static final double POSITION_SHOULDER_kF = 0.0;
    private static final double POSITION_SHOULDER_TOLERANCE = 0.01;
    private static final double POSITION_SHOULDER_LOWER_LIMIT = Math.min(ArmState.Stowed.shoulderPosition, ArmState.High.shoulderPosition);
    private static final double POSITION_SHOULDER_UPPER_LIMIT = Math.max(ArmState.Stowed.shoulderPosition, ArmState.High.shoulderPosition);
    private static final boolean POSITION_SHOULDER_SOFT_LIMITS = true;
    private static final boolean POSITION_SHOULDER_SENSOR_PHASE = false;
    private static final boolean POSITION_SHOULDER_INVERT_MOTOR = false;

    // Arm shoulder position PID config
    public static final SparkPIDConfig POSITION_SHOULDER_CONFIG = new SparkPIDConfig(
      POSITION_SHOULDER_SENSOR_PHASE,
      POSITION_SHOULDER_INVERT_MOTOR,
      POSITION_SHOULDER_kP,
      POSITION_SHOULDER_kI,
      POSITION_SHOULDER_kD,
      POSITION_SHOULDER_kF,
      POSITION_SHOULDER_TOLERANCE,
      POSITION_SHOULDER_LOWER_LIMIT,
      POSITION_SHOULDER_UPPER_LIMIT,
      POSITION_SHOULDER_SOFT_LIMITS
    );

    // Arm elbow motion settings
    private static final double MOTION_ELBOW_VELOCITY = 1.5; 
    private static final double MOTION_ELBOW_ACCELERATION = 1.0;
    public static final Constraints MOTION_ELBOW_CONTRAINT = new TrapezoidProfile.Constraints(MOTION_ELBOW_VELOCITY, MOTION_ELBOW_ACCELERATION);


    // Arm elbow position PID settings
    private static final double POSITION_ELBOW_kP = 3.5; 
    private static final double POSITION_ELBOW_kI = 0.0;
    private static final double POSITION_ELBOW_kD = 8.0; 
    private static final double POSITION_ELBOW_kF = 0.0;
    private static final double POSITION_ELBOW_TOLERANCE = 0.01;
    private static final double POSITION_ELBOW_LOWER_LIMIT = Math.min(ArmState.Stowed.elbowPosition, ArmState.High.elbowPosition);
    private static final double POSITION_ELBOW_UPPER_LIMIT = Math.max(ArmState.Stowed.elbowPosition, ArmState.High.elbowPosition);
    private static final boolean POSITION_ELBOW_SOFT_LIMITS = true;
    private static final boolean POSITION_ELBOW_SENSOR_PHASE = false;
    private static final boolean POSITION_ELBOW_INVERT_MOTOR = false;

    // Arm elbow position PID config
    public static final SparkPIDConfig POSITION_ELBOW_CONFIG = new SparkPIDConfig(
      POSITION_ELBOW_SENSOR_PHASE,
      POSITION_ELBOW_INVERT_MOTOR,
      POSITION_ELBOW_kP,
      POSITION_ELBOW_kI,
      POSITION_ELBOW_kD,
      POSITION_ELBOW_kF,
      POSITION_ELBOW_TOLERANCE,
      POSITION_ELBOW_LOWER_LIMIT,
      POSITION_ELBOW_UPPER_LIMIT,
      POSITION_ELBOW_SOFT_LIMITS
    );
  }

  public static class Intake {
    public static final double INTAKE_SPIN_MOTOR_SPEED = 1.0;
    public static final double OUTTAKE_SPIN_MOTOR_SPEED = -0.5;

  }

  public static class DriveHardware {
    public static final int FRONT_LEFT_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int REAR_LEFT_MOTOR_ID = 4;
    public static final int REAR_RIGHT_MOTOR_ID = 5;
  }

  public static class ArmHardware {
    public static final int ARM_SHOULDER_MASTER_MOTOR_ID = 6;
    public static final int ARM_SHOULDER_SLAVE_MOTOR_ID = 7;
    public static final int ARM_ELBOW_MOTOR_ID = 8;
  }

  public static class IntakeHardware {
    public static final int ROLLER_MOTOR_ID = 9;
  }

  public static class AccessoryHardware {
    public static final int BLINKIN_LED_CONTROLLER_PORT = 0;
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static class Auto {
    public static final String EVENT_MAP_INTAKE = "intake";
    public static final String EVENT_MAP_OUTAKE = "outake";
    public static final String EVENT_MAP_PRINT = "print";
    public static final String EVENT_MAP_STOWED = "stowed";
    public static final String EVENT_MAP_GROUND = "ground";
    public static final String EVENT_MAP_MIDDLE = "middle";
    public static final String EVENT_MAP_HIGH = "high";
  }
}
