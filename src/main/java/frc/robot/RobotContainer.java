// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.BottomObject;
import frc.robot.commands.autonomous.BottomObjectScore;
import frc.robot.commands.autonomous.BottomPadObject;
import frc.robot.commands.autonomous.MidObjectA;
import frc.robot.commands.autonomous.MidObjectAScore;
import frc.robot.commands.autonomous.MidObjectBScore;
import frc.robot.commands.autonomous.MidPad;
import frc.robot.commands.autonomous.MidPadObject;
import frc.robot.commands.autonomous.TestAuto;
import frc.robot.commands.autonomous.TopObject;
import frc.robot.commands.autonomous.TopObjectScore;
import frc.robot.commands.autonomous.TopPadObjectAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.BlinkinLEDController;
import frc.robot.utils.SparkPIDConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  private static final boolean REAL_HARDWARE = true;

  // The robot's subsystems and commands are defined here...
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(DriveSubsystem.initializeHardware(REAL_HARDWARE),
                                                                           Constants.Drive.DRIVE_TURN_PID,
                                                                           Constants.Drive.DRIVE_BALANCE_PID,
                                                                           Constants.HID.CONTROLLER_DEADBAND,
                                                                           Constants.Drive.DRIVE_SLIP_RATIO,
                                                                           Constants.Drive.DRIVE_TURN_SCALAR,
                                                                           Constants.Drive.DRIVE_LOOKAHEAD,
                                                                           Constants.Drive.DRIVE_TRACTION_CONTROL_CURVE,
                                                                           Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
                                                                           Constants.Drive.DRIVE_TURN_INPUT_CURVE);
  private static final ArmSubsystem ARM_SUBSYSTEM = new ArmSubsystem(ArmSubsystem.initializeHardware(REAL_HARDWARE),
                                                                          new Pair<SparkPIDConfig,SparkPIDConfig>(
                                                                            Constants.Arm.MOTION_SHOULDER_CONFIG,
                                                                            Constants.Arm.POSITION_SHOULDER_CONFIG
                                                                          ), 
                                                                          new Pair<SparkPIDConfig, SparkPIDConfig>(
                                                                            Constants.Arm.MOTION_ELBOW_CONFIG,
                                                                            Constants.Arm.POSITION_ELBOW_CONFIG
                                                                          )
                                                                    );
  private static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem(IntakeSubsystem.initializeHardware(REAL_HARDWARE));

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
  private void configureBindings() {
    PRIMARY_CONTROLLER.start().onTrue(new InstantCommand(() -> DRIVE_SUBSYSTEM.toggleTractionControl()));
    PRIMARY_CONTROLLER.a().onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.setArmState(ArmState.Stowed)));
    PRIMARY_CONTROLLER.b().onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.setArmState(ArmState.Ground)));
    PRIMARY_CONTROLLER.x().onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.setArmState(ArmState.Middle)));
    PRIMARY_CONTROLLER.y().onTrue(new InstantCommand(() -> ARM_SUBSYSTEM.setArmState(ArmState.High)));

    PRIMARY_CONTROLLER.rightTrigger().onTrue(new InstantCommand(() -> INTAKE_SUBSYSTEM.intake()));
    PRIMARY_CONTROLLER.rightTrigger().onFalse(new InstantCommand(() -> INTAKE_SUBSYSTEM.stop()));
    PRIMARY_CONTROLLER.leftTrigger().onTrue(new InstantCommand(() -> INTAKE_SUBSYSTEM.outake()));
    PRIMARY_CONTROLLER.leftTrigger().onFalse(new InstantCommand(() -> INTAKE_SUBSYSTEM.stop()));
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("Bottom object", new BottomObject(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Bottom object and score", new BottomObjectScore(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Bottom, pad, and object", new BottomPadObject(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Middle ObjectA", new MidObjectA(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Middle ObjectA & Score", new MidObjectAScore(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Middle ObjectB & Score", new MidObjectBScore(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Middle and Charging Station", new MidPad(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Mid, Pad, & Object", new MidPadObject(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Top Object", new TopObject(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Top Object & Score", new TopObjectScore(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Top, Pad, Object", new TopPadObjectAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Test", new TestAuto(DRIVE_SUBSYSTEM));
  }

  /**
   * Initialize robot state
   */
  public void robotInit() {
    // Initialize subsystems
    BlinkinLEDController.getInstance().setTeamColor();
  }

  /**
   * Intialize robot for autonomous
   */
  public void autonomousInit() {
    DRIVE_SUBSYSTEM.autonomousInit();
    BlinkinLEDController.getInstance().setAllianceColorSolid();
  }

  /**
   * Initialize robot for teleop
   */
  public void teleopInit() {
    DRIVE_SUBSYSTEM.teleopInit();
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
