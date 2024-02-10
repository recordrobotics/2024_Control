// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;
import frc.robot.subsystems.NavSensor;
import frc.robot.utils.drivemodes.AutoOrient;
import frc.robot.utils.drivemodes.DefaultDrive;
import frc.robot.utils.drivemodes.TabletDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  // Creates Drivetrain and Controls variables
  private Drivetrain _drivetrain;
  private DoubleControl _controls;

  // Creates field display var
  private Field2d m_field = new Field2d();

  // Sets up sendable chooser for drivemode
  public enum DriveMode {Robot, Field, Tablet}
  private SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

  // Sets up variables for 
  public AutoOrient autoOrient = new AutoOrient();
  public DefaultDrive defaultDrive = new DefaultDrive();
  public TabletDrive tabletDrive = new TabletDrive();

  /**
   * @param drivetrain
   */
  public ManualSwerve(Drivetrain drivetrain, NavSensor nav, DoubleControl controls) {

    // Init variables
    _drivetrain = drivetrain;
    _controls = controls;
    addRequirements(drivetrain);

    // Creates selector on SmartDashboard for drivemode
    //driveMode.addOption("AutoOrient", DriveMode.AutoOrient);
    driveMode.addOption("Robot", DriveMode.Robot);
    driveMode.addOption("Field", DriveMode.Field);
    driveMode.addOption("Tablet", DriveMode.Tablet);
    driveMode.setDefaultOption("Field", DriveMode.Field);


    // puts 2d field data on Smartdashboard
    SmartDashboard.putData("Field", m_field);
    
    // puts selector data on Smartdashboard
    SmartDashboard.putData(driveMode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Target Velocity and Angle
     */

    // Gets swerve position and sets to field position
    _drivetrain.updatePoseFilter();
    Pose2d swerve_position = _drivetrain.poseFilter.getEstimatedPosition();
    m_field.setRobotPose(swerve_position);


    // Puts on shuffleboard
    SmartDashboard.putNumber("Rotation", swerve_position.getRotation().getDegrees());
    SmartDashboard.putNumber("X", swerve_position.getX());
    SmartDashboard.putNumber("Y", swerve_position.getY());


    // Control to reset pose if reset button is pressed
    if (_controls.getResetPressed()) {
      _drivetrain.resetPose();
    }

    // Sets up driveCommandData object
    DriveCommandData driveCommandData;

    // Auto-orient function
    if (autoOrient.shouldExecute(_controls)) {
      driveCommandData = autoOrient.calculate(_controls, swerve_position);
    }

    // Tablet Drive
    else if (driveMode.getSelected() == DriveMode.Tablet) {
      driveCommandData = tabletDrive.calculate(_controls, swerve_position);
    }
    
    // DefualtChassisRelative
    else if (driveMode.getSelected() == DriveMode.Robot) {
      driveCommandData = defaultDrive.calculate(_controls, swerve_position, false);
    }

    // DefaultFieldRelative, which is also default
    else {
      driveCommandData = defaultDrive.calculate(_controls, swerve_position, true);
    }

    // Drive command
    _drivetrain.drive(driveCommandData);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}