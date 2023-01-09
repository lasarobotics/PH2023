// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.BlinkinLEDController;
import frc.robot.utils.DataLogger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final boolean REAL_HARDWARE = true;

  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(REAL_HARDWARE),
                                                                           Constants.HID.CONTROLLER_DEADBAND,
                                                                           Constants.Drive.DRIVE_SLIP_RATIO,
                                                                           Constants.Drive.DRIVE_kP,
                                                                           Constants.Drive.DRIVE_kD, 
                                                                           Constants.Drive.DRIVE_TURN_SCALAR,
                                                                           Constants.Drive.DRIVE_LOOKAHEAD,
                                                                           Constants.Drive.DRIVE_TRACTION_CONTROL_CURVE,
                                                                           Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
                                                                           Constants.Drive.DRIVE_TURN_INPUT_CURVE);

  // Controllers
  private static final CommandXboxController PRIMARY_CONTROLLER = 
    new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);
  private static final CommandXboxController SECONDARY_CONTROLLER =
    new CommandXboxController(Constants.HID.SECONDARY_CONTROLLER_PORT);

  private static SendableChooser<SequentialCommandGroup> m_automodeChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands
    DRIVE_SUBSYSTEM.setDefaultCommand(
      new RunCommand(
        () -> DRIVE_SUBSYSTEM.teleopPID(PRIMARY_CONTROLLER.getLeftY(), PRIMARY_CONTROLLER.getRightX()),
        DRIVE_SUBSYSTEM
      )
    );

    // Configure the trigger bindings
    configureBindings();

    // Configure ShuffleBoard
    defaultShuffleboardTab();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.addOption("Do nothing", new SequentialCommandGroup());
  }

  /**
   * Initialize robot state
   */
  public void robotInit() {
    // Initialize subsystems and start logging
    DataLogger.getInstance().startLogging();
    CommandScheduler.getInstance().schedule(DataLogger.LOGGING_COMMAND);
    BlinkinLEDController.getInstance().setTeamColor();
  }

  /**
   * Intialize robot for autonomous
   */
  public void autonomousInit() {
    BlinkinLEDController.getInstance().setAllianceColorSolid();
  }

  /**
   * Initialize robot for teleop
   */
  public void teleopInit() {
    BlinkinLEDController.getInstance().setAllianceColorSolid();
  }

  /**
   * Initialize robot for disable
   */
  public void disabledInit() {
    BlinkinLEDController.getInstance().setTeamColor();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
  }
}
