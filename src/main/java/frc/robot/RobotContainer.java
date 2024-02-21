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
import frc.robot.commands.hybrid.ComplexTeleAuto;
import frc.robot.commands.manual.ManualClimbers;
import frc.robot.commands.manual.ManualCrashbar;
import frc.robot.commands.manual.ManualShooter;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
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

  private final AutoPath _autoPath;

  private List<Pair<Subsystem, Command>> _teleopPairs;
  private ManualSwerve _manualSwerve;
  private ManualShooter _manualShooter;
  private ManualClimbers _manualClimbers;
  private ManualCrashbar _manualCrashbar;
  private DoubleControl _controlInput;

  private ComplexTeleAuto _complexTeleAuto;
  private RobotKill _robotKill;

  private Command autoCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init Swerve
    _drivetrain = new Drivetrain();
    _shooter = new Shooter();
    _climbers = new Climbers();
    _crashbar = new Crashbar();
    _autoPath = new AutoPath(_drivetrain);

    // Sets up auto chooser
    _autoPath.putAutoChooser();

    // Init Nav
    NavSensor.initNav();

    // Bindings and Teleop
    initTeleopCommands();
    // configureButtonBindings();

  }

  private void initTeleopCommands() {

    // Creates teleopPairs object
    _teleopPairs = new ArrayList<>();

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _controlInput = new DoubleControl(RobotMap.Control.STICKPAD_PORT, RobotMap.Control.XBOX_PORT);

    // Adds drivetrain & manual swerve to teleop commands
    _manualSwerve = new ManualSwerve(_drivetrain, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_drivetrain, _manualSwerve));

    _manualShooter = new ManualShooter(_shooter, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_shooter, _manualShooter));

    _manualClimbers = new ManualClimbers(_climbers, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_climbers, _manualClimbers));

    _manualCrashbar = new ManualCrashbar(_crashbar, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_crashbar, _manualCrashbar));

    // Configure default bindings
    // _complexTeleAuto = new ComplexTeleAuto(_drivetrain);
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
    BooleanSupplier getTeleAutoStart = () -> _controlInput.getTeleAutoStart();
    Trigger teleAutoStartTrigger = new Trigger(getTeleAutoStart);
    teleAutoStartTrigger.toggleOnTrue(_complexTeleAuto);

    // BooleanSupplier getTeleAutoKill = () -> _controlInput.getKillAuto();
    // Trigger teleAutoKillTrigger = new Trigger(getTeleAutoKill);
    // //teleAutoStartTrigger.onTrue(_complexTeleAuto);
    // teleAutoStartTrigger.negate()

    BooleanSupplier getRobotKill = () -> _controlInput.getKillAuto();
    Trigger robotKillTrigger = new Trigger(getRobotKill);
    robotKillTrigger.whileTrue(_robotKill);
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

  public void testSwerve() {
  }
}