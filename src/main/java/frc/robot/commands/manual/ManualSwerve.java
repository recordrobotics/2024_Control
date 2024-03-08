// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import frc.robot.control.AbstractControl;
import frc.robot.control.AbstractControl.DriverOrientation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  // Creates Drivetrain and Controls variables
  private Drivetrain _drivetrain;
  public AbstractControl _controls;
  private SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();


  /**
   * @param drivetrain
   */
  public ManualSwerve(Drivetrain drivetrain, AbstractControl controls) {

    // Init variables
    _drivetrain = drivetrain;
    _controls = controls;
    addRequirements(drivetrain);

    driverOrientation.addOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
    driverOrientation.addOption("Y Axis", DriverOrientation.YAxisTowardsTrigger);
    driverOrientation.setDefaultOption("X Axis", DriverOrientation.XAxisTowardsTrigger);

    // puts selector data on Smartdashboard
    SmartDashboard.putData("Joystick Orientation", driverOrientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    AbstractControl.driverOrientation = driverOrientation.getSelected();

    // Gets swerve position and sets to field position
    Pose2d swerve_position = _drivetrain.poseFilter.getEstimatedPosition();

    // Sets up driveCommandData object
    DriveCommandData driveCommandData = _controls.getDriveCommandData(swerve_position);
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