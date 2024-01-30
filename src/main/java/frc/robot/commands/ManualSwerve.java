// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.control.IControlInput;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.NavSensor;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private Swerve _swerve;
  private IControlInput _controls;

  private Field2d m_field = new Field2d();

  public ChassisSpeeds target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwerve(Swerve swerve, NavSensor nav, IControlInput controls) {
    _swerve = swerve;
    _controls = controls;
    addRequirements(swerve);

    SmartDashboard.putData("Field", m_field);
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

    // Gets swerve position
    Pose2d swerve_position = _swerve.poseFilter.getEstimatedPosition();

    m_field.setRobotPose(swerve_position);

    // Puts on shuffleboard
    SmartDashboard.putNumber("F rot", swerve_position.getRotation().getDegrees());
    SmartDashboard.putNumber("F X", swerve_position.getX());
    SmartDashboard.putNumber("F Y", swerve_position.getY());

    

    if (_controls.getResetPressed()) {
      _swerve.resetPose();
    }

    _swerve.setTargetChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(
        _controls.getX(),
        -_controls.getY(),
        -_controls.getSpin(),
        swerve_position.getRotation()));
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
