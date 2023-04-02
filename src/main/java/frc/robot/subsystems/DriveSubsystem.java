/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPoint;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem.GameObject;
import frc.robot.utils.AutoTrajectory;
import frc.robot.utils.PIDConstants;
import frc.robot.utils.SparkMax;
import frc.robot.utils.TractionControlController;
import frc.robot.utils.TurnPIDController;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private boolean isHardwareReal;
    private SparkMax lMasterMotor, rMasterMotor;
    private SparkMax lSlaveMotor, rSlaveMotor;
    private AHRS navx;

    public Hardware(boolean isHardwareReal,
        SparkMax lMasterMotor,
        SparkMax rMasterMotor,
        SparkMax lSlaveMotor,
        SparkMax rSlaveMotor,
        AHRS navx) {
      this.isHardwareReal = isHardwareReal;
      this.lMasterMotor = lMasterMotor;
      this.rMasterMotor = rMasterMotor;
      this.lSlaveMotor = lSlaveMotor;
      this.rSlaveMotor = rSlaveMotor;
      this.navx = navx;
    }
  }

  public class GridSelector {
    private int m_grid;
    private boolean m_isBlue;

    public GridSelector() {
      m_grid = 0;
    }

    public GridSelector(int grid) {
      m_isBlue = DriverStation.getAlliance() == Alliance.Blue;
      if (m_isBlue)
        grid = 9 - grid;
      grid = Math.min(3, Math.max(1, grid));
      m_grid = grid;
    }

    /**
     * 
     * @param Grid to be set to target
     */
    public void setGrid(int grid) {
      m_isBlue = DriverStation.getAlliance() == Alliance.Blue;
      if (m_isBlue)
        grid = 9 - grid;
      grid = Math.min(3, Math.max(1, grid));
      m_grid = grid;
    }

    /**
     * 
     * @return Selected grid position (Range 1-9)
     */
    public int getGrid() {
      return m_grid;
    }
  }

  private TurnPIDController m_turnPIDController;
  private PIDController m_pitchPIDController;
  private TractionControlController m_tractionControlController;
  private DifferentialDrivePoseEstimator m_poseEstimator;
  private DifferentialDriveKinematics m_kinematics;
  private LinearFilter m_velocityFilter;

  private SparkMax m_lMasterMotor;
  private SparkMax m_lSlaveMotor;

  private SparkMax m_rMasterMotor;
  private SparkMax m_rSlaveMotor;

  private AHRS m_navx;

  private final int CURRENT_LIMIT = 55;
  private final double TOLERANCE = 0.125;
  private final double MAX_VOLTAGE = 12.0;
  private final double GRID_OFFSET_X = 0.5588;
  private final double GRID_OFFSET_Z = 1.0;
  private final double VISION_AIM_DAMPENER = 0.9;

  private final double BALANCE_OFFSET = 0.5;

  private final double MOVE_BOOST_SCALAR = 1.0;
  private final double MOVE_NORMAL_SCALAR = 0.5;

  private final double TURN_LIMIT_SCALAR = 0.7;
  private final double TURN_NORMAL_SCALAR = 1.0;

  private double m_deadband = 0.0;

  private double m_driveMultiplier = MOVE_NORMAL_SCALAR;
  private double m_turnMultiplier = TURN_NORMAL_SCALAR;

  // Drive specs, these numbers use the motor shaft encoder
  private static final double DRIVE_TRACK_WIDTH = 0.57221;
  private static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524; // 6" wheels
  private static final double DRIVE_GEAR_RATIO = 10.71;
  private static final double DRIVE_TICKS_PER_METER = (Constants.Global.NEO_ENCODER_TICKS_PER_ROTATION
      * DRIVE_GEAR_RATIO) * (1 / (DRIVE_WHEEL_DIAMETER_METERS * Math.PI));
  private static final double DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
  private static final double DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK
      * Constants.Global.NEO_ENCODER_TICKS_PER_ROTATION;
  private static final double DRIVETRAIN_EFFICIENCY = 0.9;
  private static final double DRIVE_MAX_LINEAR_SPEED = (Constants.Global.NEO_MAX_RPM / 60) * DRIVE_METERS_PER_ROTATION
      * DRIVETRAIN_EFFICIENCY;

  private static GridSelector m_gridSelector;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param drivetrainHardware   Hardware devices required by drivetrain
   * @param gridSelector         Chosen grid to use
   * @param turnPIDConstants     PID values for turning
   * @param pitchPIDConstants    PID values for balancing
   * @param deadband             Deadband for controller input [+0.001, +0.2]
   * @param slipRatio            Slip ratio for traction control [+0.01, +0.15]
   * @param kP                   Proportional gain
   * @param kD                   Derivative gain
   * @param turnScalar           Scalar for turn input (degrees)
   * @param lookAhead            Turn PID lookahead, in number of loops
   * @param metersPerTick        Meters traveled per encoder tick (meters)
   * @param maxLinearSpeed       Maximum linear speed of the robot (m/s)
   * @param tractionControlCurve Spline function characterising traction of the
   *                             robot
   * @param throttleInputCurve   Spline function characterising throttle input
   * @param turnInputCurve       Spline function characterising turn input
   */
  public DriveSubsystem(Hardware drivetrainHardware, PIDConstants turnPIDConstants, PIDConstants pitchPIDConstants,
      double deadband, double slipRatio, double turnScalar, double lookAhead,
      PolynomialSplineFunction tractionControlCurve, PolynomialSplineFunction throttleInputCurve,
      PolynomialSplineFunction turnInputCurve) {
    m_turnPIDController = new TurnPIDController(turnPIDConstants.kP, turnPIDConstants.kD, turnScalar, lookAhead,
        deadband, turnInputCurve);
    m_pitchPIDController = new PIDController(pitchPIDConstants.kP, pitchPIDConstants.kD, pitchPIDConstants.kF);
    m_tractionControlController = new TractionControlController(slipRatio, DRIVE_MAX_LINEAR_SPEED, deadband,
        tractionControlCurve, throttleInputCurve);
    m_kinematics = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH);

    this.m_lMasterMotor = drivetrainHardware.lMasterMotor;
    this.m_rMasterMotor = drivetrainHardware.rMasterMotor;
    this.m_lSlaveMotor = drivetrainHardware.lSlaveMotor;
    this.m_rSlaveMotor = drivetrainHardware.rSlaveMotor;

    this.m_navx = drivetrainHardware.navx;

    this.m_deadband = deadband;

    m_gridSelector = new GridSelector(0);

    m_velocityFilter = LinearFilter.singlePoleIIR(Constants.Global.ROBOT_LOOP_PERIOD * 3, Constants.Global.ROBOT_LOOP_PERIOD);
    
    // Reset Spark Max settings
    m_lMasterMotor.restoreFactoryDefaults();
    m_lSlaveMotor.restoreFactoryDefaults();
    m_rMasterMotor.restoreFactoryDefaults();
    m_rSlaveMotor.restoreFactoryDefaults();

    // Set all drive motors to coast
    m_lMasterMotor.setIdleMode(IdleMode.kCoast);
    m_lSlaveMotor.setIdleMode(IdleMode.kCoast);
    m_rMasterMotor.setIdleMode(IdleMode.kCoast);
    m_rSlaveMotor.setIdleMode(IdleMode.kCoast);

    // Make rear left motor controllers follow left master
    m_lSlaveMotor.follow(m_lMasterMotor);

    // Make rear right motor controllers follow right master
    m_rSlaveMotor.follow(m_rMasterMotor);

    // Only do this stuff if hardware is real
    if (drivetrainHardware.isHardwareReal) {
      // Set position and velocity conversion factor, based on gearbox output shaft
      // encoder
      double conversionFactor = DRIVE_WHEEL_DIAMETER_METERS * Math.PI;
      m_lMasterMotor.getRelativeEncoder().setPositionConversionFactor(conversionFactor);
      m_lMasterMotor.getRelativeEncoder().setVelocityConversionFactor(conversionFactor / 60);
      m_rMasterMotor.getRelativeEncoder().setPositionConversionFactor(conversionFactor);
      m_rMasterMotor.getRelativeEncoder().setVelocityConversionFactor(conversionFactor / 60);
      m_lSlaveMotor.getRelativeEncoder().setPositionConversionFactor(conversionFactor);
      m_lSlaveMotor.getRelativeEncoder().setVelocityConversionFactor(conversionFactor / 60);
      m_rSlaveMotor.getRelativeEncoder().setPositionConversionFactor(conversionFactor);
      m_rSlaveMotor.getRelativeEncoder().setVelocityConversionFactor(conversionFactor / 60);
    }

    // Invert only left side
    m_lMasterMotor.setInverted(true);
    m_lSlaveMotor.setInverted(true);
    m_rMasterMotor.setInverted(false);
    m_rSlaveMotor.setInverted(false);

    // Enable voltage compensation
    m_lMasterMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_lSlaveMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_rMasterMotor.enableVoltageCompensation(MAX_VOLTAGE);
    m_rSlaveMotor.enableVoltageCompensation(MAX_VOLTAGE);

    // Enable current limit
    m_lMasterMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_lSlaveMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_rMasterMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    m_rSlaveMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    // Initialise PID subsystem setpoint and input
    m_navx.calibrate();
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);

    // Set drive PID tolerance
    m_turnPIDController.setTolerance(TOLERANCE);

    // Initialise odometry
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics,
        new Rotation2d(),
        0.0,
        0.0,
        new Pose2d());
  }

  /**
   * Initialize hardware devices for drive subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware(boolean isHardwareReal) {
    Hardware drivetrainHardware = new Hardware(isHardwareReal,
        new SparkMax(Constants.DriveHardware.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless),
        new SparkMax(Constants.DriveHardware.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless),
        new SparkMax(Constants.DriveHardware.REAR_LEFT_MOTOR_ID, MotorType.kBrushless),
        new SparkMax(Constants.DriveHardware.REAR_RIGHT_MOTOR_ID, MotorType.kBrushless),
        new AHRS(SPI.Port.kMXP));

    return drivetrainHardware;
  }

  /**
   * Reset Drivesubsystem navX MXP yaw angle
   */
  private void resetAngle() {
    m_navx.reset();
  }

  /**
   * 
   * @param a Vec3 of Position A
   * @param b Vec3 of Position B
   * @return Euclidean distance between A and B
   */
  private double getDistance(Pose3d a, Pose3d b) {
    return Math.sqrt(
        Math.pow(a.getX() - b.getX(), 2)
            + Math.pow(a.getX() - b.getX(), 2)
            + Math.pow(a.getX() - b.getX(), 2));
  }

  /**
   * 
   * @param grid Selected grid to move to (Range 1-9)
   */
  public void setGridSelector(int grid) {
    m_gridSelector.setGrid(grid);
  }

  /**
   * Reset left and right drive encoders
   */
  public void resetEncoders() {
    m_lMasterMotor.resetRelativeEncoder();
    m_rMasterMotor.resetRelativeEncoder();
  }

  @Override
  public void periodic() {
    updateOdometry();
    smartDashboard();
  }

  /**
   * Create SmartDashboard indicators
   */
  public void smartDashboard() {
    SmartDashboard.putBoolean("TC", m_tractionControlController.isEnabled());
  }

  /**
   * Initialize drive subsystem for autonomous
   */
  public void autonomousInit() {
    // Set all drive motors to coast
    m_lMasterMotor.setIdleMode(IdleMode.kCoast);
    m_lSlaveMotor.setIdleMode(IdleMode.kCoast);
    m_rMasterMotor.setIdleMode(IdleMode.kCoast);
    m_rSlaveMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Initialize drive subsystem for teleop
   */
  public void teleopInit() {
    // Set all drive motors to coast
    m_lMasterMotor.setIdleMode(IdleMode.kCoast);
    m_lSlaveMotor.setIdleMode(IdleMode.kCoast);
    m_rMasterMotor.setIdleMode(IdleMode.kCoast);
    m_rSlaveMotor.setIdleMode(IdleMode.kCoast);

    // Reset drive PID
    resetDrivePID();
  }

  /**
   * Call this repeatedly to drive without PID during teleoperation
   * 
   * @param speed Desired speed [-1.0, +1.0]
   * @param turn  Turn input [-1.0, +1.0]
   * @param power exponent for drive response curve. 1 is linear response
   */
  public void teleop(double speed, double turn, int power) {
    speed = Math.copySign(Math.pow(speed, power), speed);
    turn = Math.copySign(Math.pow(turn, power), turn);

    speed = MathUtil.applyDeadband(speed, m_deadband);
    turn = MathUtil.applyDeadband(turn, m_deadband);

    m_lMasterMotor.set(speed, ControlType.kDutyCycle, -turn, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(speed, ControlType.kDutyCycle, +turn, ArbFFUnits.kPercentOut);
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * 
   * @param speedRequest Desired speed [-1.0, +1.0]
   * @param turnRequest  Turn input [-1.0, +1.0]
   */
  public void teleopPID(double speedRequest, double turnRequest) {
    // Multiply speed by speed scalar
    speedRequest *= m_driveMultiplier;
    turnRequest *= m_turnMultiplier;

    // Calculate next speed output
    double speedOutput = m_tractionControlController.calculate(getInertialVelocity(), speedRequest,
        getAverageWheelSpeed(), isTurning());

    // Calculate next PID turn output
    double turnOutput = m_turnPIDController.calculate(getAngle(), getTurnRate(), turnRequest);

    // Run motors with appropriate values
    m_lMasterMotor.set(speedOutput, ControlType.kDutyCycle, -turnOutput, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(speedOutput, ControlType.kDutyCycle, +turnOutput, ArbFFUnits.kPercentOut);
  }

  /**
   * Call this repeatedly to automatically balance robot in pitch orientation
   */
  public void autoBalance() {
    double pitchOutput = m_pitchPIDController.calculate(getPitch());

    m_lMasterMotor.set(pitchOutput, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(pitchOutput, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  /**
   * Enable BOOSTTTT speed
   */
  public void enableBoost() {
    m_driveMultiplier = MOVE_BOOST_SCALAR;
  }

  /**
   * Disable BOOOOOST speed
   */
  public void disableBoost() {
    m_driveMultiplier = MOVE_NORMAL_SCALAR;
  }


  /**
   * Enable turn slowdown
   */
  public void enableTurnRateLimit() {
    m_turnMultiplier = TURN_LIMIT_SCALAR;
  }

  /**
   * Disable turn slowdown
   */
  public void disableTurnRateLimit() {
    m_turnMultiplier = TURN_NORMAL_SCALAR;
  }


  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_tractionControlController.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_tractionControlController.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_tractionControlController.disableTractionControl();
  }

  /**
   * Turn robot by angleDelta
   * 
   * @param angleDelta Degrees to turn robot by
   */
  public void aimToAngle(double angleDelta) {
    aimToAngle(angleDelta, 0.0);
  }

  /**
   * Turn robot by angleDelta
   * 
   * @param angleDelta   Degrees to turn robot by
   * @param speedRequest Desired speed [-1.0, +1.0]
   */
  public void aimToAngle(double angleDelta, double speedRequest) {
    angleDelta *= VISION_AIM_DAMPENER;
    m_turnPIDController.setSetpoint(getAngle() + angleDelta);

    // Calculate next speed output
    double speedOutput = m_tractionControlController.calculate(getInertialVelocity(), speedRequest,
        getAverageWheelSpeed(), isTurning());

    double turnOutput = m_turnPIDController.calculate(getAngle());
    m_lMasterMotor.set(speedOutput, ControlType.kDutyCycle, -turnOutput, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(speedOutput, ControlType.kDutyCycle, +turnOutput, ArbFFUnits.kPercentOut);
  }

  /**
   * Whether robot is aimed at target
   * 
   * @param tolerance tolerance in degrees
   * @return true if robot is aimed at target within tolerance
   */
  public boolean isOnTarget() {
    return m_turnPIDController.atSetpoint();
  }

  /**
   * Maintain setpoint angle
   */
  public void maintainAngle() {
    double turnOutput = m_turnPIDController.calculate(getAngle());
    m_lMasterMotor.set(0.0, ControlType.kDutyCycle, -turnOutput, ArbFFUnits.kPercentOut);
    m_rMasterMotor.set(0.0, ControlType.kDutyCycle, +turnOutput, ArbFFUnits.kPercentOut);
  }

  /**
   * Controls the left and right sides of the drive directly using velocities
   * <p>
   * Only use this method to drive during autonomous!
   * @param leftVelocity Left side linear velocity
   * @param rightVelocity Right side linear velocity
   */
  public void autoDrive(double leftVelocity, double rightVelocity) {
    m_lMasterMotor.set(m_tractionControlController.lookupVelocity(leftVelocity), ControlType.kDutyCycle);
    m_rMasterMotor.set(m_tractionControlController.lookupVelocity(rightVelocity), ControlType.kDutyCycle);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * <p>
   * Only use this method to drive during autonomous!
   * 
   * @param leftVoltage  Left side output voltage
   * @param rightVoltage Right side output voltage
   */
  public void autoDriveVoltage(double leftVoltage, double rightVoltage) {
    m_lMasterMotor.setVoltage(leftVoltage);
    m_rMasterMotor.setVoltage(rightVoltage);
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_lMasterMotor.getRelativeEncoderVelocity(),
        m_rMasterMotor.getRelativeEncoderVelocity());
  }

  /**
   * Return current average wheel speeds of the robot
   * 
   * @return
   */
  public double getAverageWheelSpeed() {
    return Math.abs((m_lMasterMotor.getRelativeEncoderVelocity() + m_rMasterMotor.getRelativeEncoderVelocity()) / 2);
  }

  /**
   * Resets the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(m_navx.getRotation2d(), 0.0, 0.0, pose);
  }

  /**
   * Update robot odometry
   * <p>
   * Repeatedly call this method at a steady rate to keep track of robot position
   */
  public void updateOdometry() {
    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()),
        m_lMasterMotor.getEncoderPosition(),
        m_rMasterMotor.getEncoderPosition());
    Pair<Pose2d, Double> result = VisionSubsystem.getInstance().getEstimatedGlobalPose(getPose());
    Pose2d camPose = result.getFirst();
    double camPoseObsTime = result.getSecond();
    if (camPose != null)
      m_poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
  }

  /**
   * Returns the currently estimated pose of the robot
   * <p>
   * This method is called periodically by the Ramsete command to update and
   * obtain the latest pose
   * 
   * @return The pose
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns track width of the robot
   * 
   * @return track width in meters
   */
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Returns the turn rate of the robot.
   * 
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate();
  }

  /**
   * Returns inertial velocity of the robot.
   * 
   * @return Velocity of the robot as measured by the NAVX
   */
  public double getInertialVelocity() {
    return m_velocityFilter.calculate(m_navx.getVelocityY());
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    m_lMasterMotor.stopMotor();
    m_rMasterMotor.stopMotor();
  }

  /**
   * Get whether robot is turning or not
   * 
   * @return true if robot is turning
   */
  public boolean isTurning() {
    return m_turnPIDController.isTurning();
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * 
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Get DriveSubsystem pitch as detected by the navX MXP
   * 
   * @return Current pitch angle
   */
  public double getPitch() {
    return m_navx.getRoll() + BALANCE_OFFSET;
  }

  /**
   * Reset DriveSubsystem PID
   */
  public void resetDrivePID() {
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);
    m_turnPIDController.reset();
  }

  /**
   * Get setpoint for drive PID
   * 
   * @return current setpoint in degrees
   */
  public double getDrivePIDSetpoint() {
    return m_turnPIDController.getSetpoint();
  }

  /**
   * 
   * @param grid       Grid labelled 1-9 here
   *                   https://firstfrc.blob.core.windows.net/frc2023/Manual/2023FRCGameManual.pdf
   *                   Page 40
   * @param gameObject Cube or Cone Enum
   * @return Command consisting of waypoints orienting and repositioning robot
   */
  public Command moveToClosestTarget(GameObject gameObject) {

    if (gameObject == null)
      return new InstantCommand();
    int grid = Math.min(3, Math.max(m_gridSelector.getGrid(), 1));

    AprilTagFieldLayout fieldLayout = VisionSubsystem.getInstance().getAprilTagFieldLayout();

    Optional<Pose3d> targetPosition = fieldLayout.getTagPose(grid);

    Pose3d[] targets = new Pose3d[] {
        new Pose3d(targetPosition.get().getX() - GRID_OFFSET_X, targetPosition.get().getY(),
            targetPosition.get().getZ() + GRID_OFFSET_Z, targetPosition.get().getRotation()),
        new Pose3d(targetPosition.get().getX(), targetPosition.get().getY(),
            targetPosition.get().getZ() + GRID_OFFSET_Z, targetPosition.get().getRotation()),
        new Pose3d(targetPosition.get().getX() + GRID_OFFSET_X, targetPosition.get().getY(),
            targetPosition.get().getZ() + GRID_OFFSET_Z, targetPosition.get().getRotation())
    };

    Pose3d goToLocation = targets[0];
    Pose3d currentPosition = new Pose3d(getPose());

    if (gameObject.equals(GameObject.Cube))
      goToLocation = targets[1];
    else {
      if (getDistance(currentPosition, targets[0]) >= getDistance(currentPosition, targets[2]))
        goToLocation = targets[2];
    }

    List<PathPoint> waypoints = List.of(
        new PathPoint(new Translation2d(currentPosition.getX(), currentPosition.getY()),
            new Rotation2d(currentPosition.getRotation().getX(), currentPosition.getRotation().getY())),
        new PathPoint(new Translation2d(goToLocation.getX(), goToLocation.getY()),
            new Rotation2d(-goToLocation.getRotation().getX(), -goToLocation.getRotation().getY())));

    return new AutoTrajectory(this, waypoints, false, getInertialVelocity(), 0.5).getCommandAndStop();
  }

  @Override
  public void close() {
    m_lMasterMotor.close();
    m_rMasterMotor.close();
    m_lSlaveMotor.close();
    m_rSlaveMotor.close();
    m_navx.close();
  }
}
