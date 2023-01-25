// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import frc.robot.utils.PIDConstants;
import frc.robot.utils.SparkPIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(0.02, 0.0, 0.0004, 0.0);
    public static final PIDConstants DRIVE_BALANCE_PID = new PIDConstants(0.02, 0.0, 0.0004, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.08;
    public static final double DRIVE_TURN_SCALAR = 25.0;
    public static final double DRIVE_LOOKAHEAD = 16;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.5, 1.0 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 2.0, 3.95 };
    private static final double DRIVE_TRACTION_CONTROL_CURVE_X[] = { 0.0, 2.0, 3.95 };
    private static final double DRIVE_TRACTION_CONTROL_CURVE_Y[] = { 0.0, 0.5, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TRACTION_CONTROL_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TRACTION_CONTROL_CURVE_X, DRIVE_TRACTION_CONTROL_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  }

  public static class Arm {
    // Arm Pivot PID config
    private static final double PIVOT_kP = 1.0;
    private static final double PIVOT_kD = 0.0;
    private static final double PIVOT_kI = 0.0;
    private static final double PIVOT_kF = 0.0;
    private static final double PIVOT_TOLERANCE = 10;
    private static final double PIVOT_LOWER_LIMIT = 0;
    private static final double PIVOT_UPPER_LIMIT = 3100;
    private static final double PIVOT_VELOCITY = Global.NEO_MAX_RPM;
    private static final double PIVOT_ACCELERATION = Global.NEO_MAX_RPM;
    private static final boolean PIVOT_SOFT_LIMITS = true;
    private static final boolean PIVOT_SENSOR_PHASE = false;
    private static final boolean PIVOT_INVERT_MOTOR = false;
    private static final AccelStrategy PIVOT_ACCEL_STRATEGY = AccelStrategy.kTrapezoidal;
  
    
    // Arm Pivot PID config
    public static final SparkPIDConfig PIVOT_CONFIG = new SparkPIDConfig(PIVOT_SENSOR_PHASE, 
                                                                         PIVOT_INVERT_MOTOR, 
                                                                         PIVOT_kP,
                                                                         PIVOT_kI,
                                                                         PIVOT_kD,
                                                                         PIVOT_kF,
                                                                         PIVOT_TOLERANCE,
                                                                         PIVOT_LOWER_LIMIT,
                                                                         PIVOT_UPPER_LIMIT,
                                                                         PIVOT_SOFT_LIMITS,
                                                                         PIVOT_VELOCITY,
                                                                         PIVOT_ACCELERATION,
                                                                         PIVOT_ACCEL_STRATEGY);

    // Arm Telescope PID config
    private static final double TELESCOPE_kP = 1.0;
    private static final double TELESCOPE_kI = 0.0;
    private static final double TELESCOPE_kD = 0.0;
    private static final double TELESCOPE_kF = 0.0;
    private static final double TELESCOPE_TOLERANCE = 10;
    private static final double TELESCOPE_LOWER_LIMIT = 0;
    private static final double TELESCOPE_UPPER_LIMIT = 3100;
    private static final double TELESCOPE_VELOCITY = Global.NEO_MAX_RPM;
    private static final double TELESCOPE_ACCELERATION = Global.NEO_MAX_RPM;
    private static final boolean TELESCOPE_SOFT_LIMITS = true;
    private static final boolean TELESCOPE_SENSOR_PHASE = false;
    private static final boolean TELESCOPE_INVERT_MOTOR = false;
    private static final AccelStrategy TELESCOPE_ACCEL_STRATEGY = AccelStrategy.kTrapezoidal;
  
    
    // Arm Telescope PID config
    public static final SparkPIDConfig TELESCOPE_CONFIG = new SparkPIDConfig(TELESCOPE_SENSOR_PHASE, 
                                                                             TELESCOPE_INVERT_MOTOR, 
                                                                             TELESCOPE_kP,
                                                                             TELESCOPE_kI,
                                                                             TELESCOPE_kD,
                                                                             TELESCOPE_kF,
                                                                             TELESCOPE_TOLERANCE,
                                                                             TELESCOPE_LOWER_LIMIT,
                                                                             TELESCOPE_UPPER_LIMIT,
                                                                             TELESCOPE_SOFT_LIMITS,
                                                                             TELESCOPE_VELOCITY,
                                                                             TELESCOPE_ACCELERATION,
                                                                             TELESCOPE_ACCEL_STRATEGY);
  }

  public static class DriveHardware {
    public static final int FRONT_LEFT_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int REAR_LEFT_MOTOR_ID = 4;
    public static final int REAR_RIGHT_MOTOR_ID = 5;
  }

  public static class ArmHardware {
    public static final int ARM_PIVOT_SHOULDER_ID = 6;
    public static final int ARM_PIVOT_ELBOW_ID = 7;
  }




  public static class AccessoryHardware {
    public static final int BLINKIN_LED_CONTROLLER_PORT = 0;
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }
}
