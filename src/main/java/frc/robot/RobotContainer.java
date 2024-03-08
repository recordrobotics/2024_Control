// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Crashbar;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.manual.ManualAcquisition;

import frc.robot.commands.hybrid.NoteOrient;
import frc.robot.commands.manual.ManualClimbers;
import frc.robot.commands.manual.ManualCrashbar;
import frc.robot.commands.manual.ManualShooter;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.manual.ManualReverse;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.commands.notes.ShootAmp;
import frc.robot.commands.notes.ShootSpeaker;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Vision _vision;

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
  private DoubleControl _controlInput;

  // Smart Commands
  private AcquireSmart _acquire;
  private ShootSpeaker _shootSpeaker;
  private ShootAmp _shootAmp;

  private NoteOrient _noteOrient;

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
    _vision = new Vision();

    // Sets up auto chooser
    _autoPath = new AutoPath(_drivetrain, _acquisition, _photosensor, _channel, _shooter, _crashbar);
    _autoPath.putAutoChooser();

    // Bindings and Teleop
    initTeleopCommands();
    configureButtonBindings();
  }

  private void initTeleopCommands() {

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _controlInput = new DoubleControl(RobotMap.Control.STICKPAD_PORT, RobotMap.Control.XBOX_PORT);
    
    // Adds default drivetrain & manual swerve to teleop commands
    _manualSwerve = new ManualSwerve(_drivetrain, _controlInput);

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

    BooleanSupplier getNoteOrient = () -> _controlInput.getTeleAutoStart();
    Trigger noteOrientTrigger = new Trigger(getNoteOrient);
    noteOrientTrigger.toggleOnTrue(_noteOrient);

    // BooleanSupplier getTeleAutoKill = () -> _controlInput.getKillAuto();
    // Trigger teleAutoKillTrigger = new Trigger(getTeleAutoKill);
    // //teleAutoStartTrigger.onTrue(_complexTeleAuto);
    // teleAutoStartTrigger.negate()
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