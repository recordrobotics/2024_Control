// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.MoveDistance;
import frc.robot.commands.ManualClimbers;
import frc.robot.commands.ManualCrashbar;
import frc.robot.commands.ManualShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Crashbar;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManualSwerve;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
  private Drivetrain _drivetrain;
  private Shooter _shooter;
  private Crashbar _crashbar;
  private Climbers _climbers;
  private List<Pair<Subsystem, Command>> _teleopPairs;
  private ManualSwerve _manualSwerve;
  private ManualShooter _manualShooter;
  private ManualClimbers _manualClimbers;
  private ManualCrashbar _manualCrashbar;

  private DoubleControl _controlInput;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init Swerve
    _drivetrain = new Drivetrain();
    _shooter = new Shooter();
    _climbers = new Climbers();
    _crashbar = new Crashbar();

    // Init Nav
    NavSensor.initNav();

    // Bindings and Teleop
    configureButtonBindings();
    initTeleopCommands();
  }

  private void initTeleopCommands() {

    // Creates teleopPairs object
    _teleopPairs = new ArrayList<>();

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _controlInput = new DoubleControl(RobotMap.Control.STICKPAD_PORT, RobotMap.Control.GAMEPAD_PORT);

    // Adds drivetrain & manual swerve to teleop commands
    _manualSwerve = new ManualSwerve(_drivetrain, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_drivetrain, _manualSwerve));

    _manualShooter = new ManualShooter(_shooter, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_shooter, _manualShooter));

    _manualClimbers = new ManualClimbers(_climbers, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_climbers, _manualClimbers));

    _manualCrashbar = new ManualCrashbar(_crashbar, _controlInput);
    _teleopPairs.add(new Pair<Subsystem, Command>(_crashbar, _manualCrashbar));
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

    // TODO: look into command binding
    /**
     * CommandJoystick joystick = new
     * CommandJoystick(RobotMap.Control.STICKPAD_PORT);
     * Trigger getReset = new Trigger(joystick.button(2));
     * getReset.onTrue(_drivetrain());
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new MoveDistance(_drivetrain, new Translation2d(1, 1)).andThen(() -> {
      System.out.println("AutoDrive completed");
    }, _drivetrain);
  }

  public void testSwerve() {
  }
}