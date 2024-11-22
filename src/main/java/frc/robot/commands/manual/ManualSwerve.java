// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.AbstractControl;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseTracker;
import frc.robot.utils.DriveCommandData;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Creates Drivetrain and Controls variables
  private Drivetrain drivetrain;
  
  private PoseTracker poseTracker;
  
  /**
   * @param drivetrain
   */
  public ManualSwerve(Drivetrain drivetrain, PoseTracker poseTracker) {
    this.poseTracker = poseTracker;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    addRequirements(poseTracker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    AbstractControl controls = ShuffleboardUI.Overview.getControl();

    DriveCommandData driveCommandData = controls.getDriveCommandData();
    drivetrain.drive(driveCommandData, poseTracker.getEstimatedPosition().getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
