// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Crashbar;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.manual.ManualAcquisition;
import frc.robot.commands.manual.ManualClimbers;
import frc.robot.commands.manual.ManualCrashbar;
import frc.robot.commands.manual.ManualShooter;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.manual.ManualReverse;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.commands.notes.ShootAmp;
import frc.robot.commands.notes.ShootSpeaker;
import frc.robot.control.AbstractControl;
import frc.robot.utils.ShuffleboardChoosers;
import frc.robot.control.DoubleXbox;
import frc.robot.control.JoystickXbox;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here
  private final Drivetrain _drivetrain;
  private final Shooter _shooter;
  private final Crashbar _crashbar;
  private final Climbers _climbers;
  private final Acquisition _acquisition;
  private final Channel _channel;
  private final Photosensor _photosensor;

  // Autonomous
  private final AutoPath _autoPath;
  private Command autoCommand;

  // Manual (default) commands
  private ManualSwerve _manualSwerve;
  private ManualReverse _manualReverse;
  private ManualShooter _manualShootSpeaker;
  private ManualShooter _manualShootAmp;
  private ManualAcquisition _manualAcquisition;
  private ManualCrashbar _manualCrashbar;
  private ManualClimbers _manualClimbers;

  // Control
  private JoystickXbox _joystickXbox;
  private DoubleXbox _doubleXbox;
  private AbstractControl _controlInput;

  // Smart Commands
  private AcquireSmart _acquire;
  private ShootSpeaker _shootSpeaker;
  private ShootAmp _shootAmp;

  // Misc commands
  private KillSpecified _killSpecified;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init Nav
    NavSensor.initNav();

    // Init subsystems
    _drivetrain = new Drivetrain();
    _channel = new Channel();
    _acquisition = new Acquisition();
    _shooter = new Shooter();
    _crashbar = new Crashbar();
    _photosensor = new Photosensor();
    _climbers = new Climbers();

    // Sets up auto chooser
    _autoPath = new AutoPath(_drivetrain, _acquisition, _photosensor, _channel, _shooter, _crashbar);
    _autoPath.putAutoChooser();

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _joystickXbox = new JoystickXbox(RobotMap.Control.STICKPAD_PORT, RobotMap.Control.XBOX_PORT);
    _doubleXbox = new DoubleXbox(0, 1);
    // Sets up Control scheme chooser
    ShuffleboardChoosers.initialize(_doubleXbox, _joystickXbox);
    _controlInput = ShuffleboardChoosers.getDriveMode();

    // Bindings and Teleop
    initTeleopCommands();
    configureButtonBindings();
  }

  private void initTeleopCommands() {
    
    // Adds default drivetrain & manual swerve to teleop commands
    _manualSwerve = new ManualSwerve(_drivetrain);

    // Robot kill command
    _killSpecified = new KillSpecified(_drivetrain, _acquisition, _channel, _shooter, _crashbar, _climbers);

    // Sets up manual commands
    _manualAcquisition = new ManualAcquisition(_acquisition, _channel);
    _manualShootSpeaker = new ManualShooter(_shooter, ShooterStates.SPEAKER);
    _manualShootAmp = new ManualShooter(_shooter, ShooterStates.AMP);
    _manualCrashbar = new ManualCrashbar(_crashbar);
    _manualClimbers = new ManualClimbers(_climbers);

    // Sets up higher level manual notes commands
    _acquire = new AcquireSmart(_acquisition, _channel, _photosensor, _shooter);
    _shootSpeaker = new ShootSpeaker(_channel, _shooter);
    _shootAmp = new ShootAmp(_channel, _shooter, _crashbar);
    _manualReverse = new ManualReverse(_acquisition, _channel);
  }

  public void teleopInit() {
    // Sets default command for manual swerve. It is the only one right now
    _drivetrain.setDefaultCommand(_manualSwerve);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(_controlInput::getKillAuto).whileTrue(_killSpecified);

    // Smart triggers
    new Trigger(_controlInput::getAcquire).toggleOnTrue(_acquire);;
    new Trigger(_controlInput::getShootSpeaker).toggleOnTrue(_shootSpeaker);;
    new Trigger(_controlInput::getShootAmp).toggleOnTrue(_shootAmp);
    new Trigger(_controlInput::getReverse).whileTrue(_manualReverse);

    // Manual triggers
    new Trigger(_controlInput::getManualShootAmp).toggleOnTrue(_manualShootAmp);
    new Trigger(_controlInput::getManualShootSpeaker).toggleOnTrue(_manualShootSpeaker);
    new Trigger(_controlInput::getManualCrashbar).toggleOnTrue(_manualCrashbar);
    new Trigger(_controlInput::getManualAcquisition).whileTrue(_manualAcquisition);
    new Trigger(_controlInput::getManualClimbers).toggleOnTrue(_manualClimbers);

    // Reset pose trigger
    new Trigger(_controlInput::getPoseReset).onTrue(new InstantCommand(_drivetrain::resetPose));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(_drivetrain, _autoPath);
    }
    return autoCommand;
  }
}