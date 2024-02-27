// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Crashbar;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RobotKill;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.manual.ManualClimbers;
import frc.robot.commands.manual.ManualCrashbar;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.notes.Acquire;
import frc.robot.commands.notes.Reverse;
import frc.robot.commands.notes.ShootAmp;
import frc.robot.commands.notes.ShootSpeaker;
import frc.robot.commands.solenoid.ClimberDown;
import frc.robot.commands.solenoid.ClimberUp;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

  // Teleop commands
  private List<Pair<Subsystem, Command>> _teleopPairs;

  // Manual (default) commands
  private ManualSwerve _manualSwerve;
  private ManualClimbers _manualClimbers;
  private ManualCrashbar _manualCrashbar;

  //
  private DoubleControl _controlInput;

  private Acquire _acquire;
  private Reverse _reverse;
  private ShootSpeaker _shootSpeaker;
  private ShootAmp _shootAmp;
  private ClimberUp _climberUp;
  private ClimberDown _climberDown;

  private RobotKill _robotKill;

  private Command autoCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init Nav
    NavSensor.initNav();

    // Init Swerve
    _drivetrain = new Drivetrain();

    // Init note systems
    _channel = new Channel();
    _acquisition = new Acquisition();
    _shooter = new Shooter();
    _crashbar = new Crashbar();
    _photosensor = new Photosensor();

    // Init climbers
    _climbers = new Climbers();

    // Sets up auto chooser
    _autoPath = new AutoPath(_drivetrain);
    _autoPath.putAutoChooser();

    // Bindings and Teleop
    initTeleopCommands();
    configureButtonBindings();
  }

  private void initTeleopCommands() {

    // Creates teleopPairs object
    _teleopPairs = new ArrayList<>();

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _controlInput = new DoubleControl(RobotMap.Control.STICKPAD_PORT, RobotMap.Control.XBOX_PORT);

    // Adds default drivetrain & manual swerve to teleop commands
    _manualSwerve = new ManualSwerve(_drivetrain, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_drivetrain, _manualSwerve));

    /*
    _manualClimbers = new ManualClimbers(_climbers, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_climbers, _manualClimbers));

    _manualCrashbar = new ManualCrashbar(_crashbar, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_crashbar, _manualCrashbar));
    */

    // Sets up higher level manual notes commands
    _acquire = new Acquire(_acquisition, _channel, _photosensor);
    _shootSpeaker = new ShootSpeaker(_channel, _shooter);
    _shootAmp = new ShootAmp(_channel, _shooter, _crashbar);
    _reverse = new Reverse(_acquisition, _channel);
    _climberUp = new ClimberUp(_climbers);
    _climberDown = new ClimberDown(_climbers);

    // Configure bindings
    _robotKill = new RobotKill(_drivetrain);
  }

  public void teleopInit() {
    for (Pair<Subsystem, Command> pair : _teleopPairs) {
      pair.getFirst().setDefaultCommand(pair.getSecond());
    }
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

    Trigger robotKillTrigger = new Trigger(_controlInput::getKillAuto);
    robotKillTrigger.whileTrue(_robotKill);

    Trigger acquireTrigger = new Trigger(_controlInput::getAcquire);
    acquireTrigger.toggleOnTrue(_acquire);

    Trigger shootSpeakerTrigger = new Trigger(_controlInput::getShootSpeaker);
    shootSpeakerTrigger.toggleOnTrue(_shootSpeaker);

    Trigger shootAmpTrigger = new Trigger(_controlInput::getShootAmp);
    shootAmpTrigger.toggleOnTrue(_shootAmp);

    Trigger reverseTrigger = new Trigger(_controlInput::getReverse);
    reverseTrigger.whileTrue(_reverse);

    BooleanSupplier getClimberUp = () -> _controlInput.getClimberUp();
    Trigger ClimberUpTrigger = new Trigger(getClimberUp);
    ClimberUpTrigger.onTrue(_climberUp);

    BooleanSupplier getClimberDown = () -> _controlInput.getClimberDown();
    Trigger ClimberDownTrigger = new Trigger(getClimberDown);
    ClimberDownTrigger.onTrue(_climberDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(_drivetrain, _autoPath).andThen(() -> {
        _drivetrain.stop();
        System.out.println("ContainerAuto End");
      }, _drivetrain);
    }
    return autoCommand;
  }
}