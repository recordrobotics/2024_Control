// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.control.IControlInput;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import frc.robot.utils.AutoOrient;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  public enum FieldReferenceFrame {
    Field,
    Robot
  }

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private Drivetrain _drivetrain;
  private IControlInput _controls;

  private Field2d m_field = new Field2d();
  private SendableChooser<FieldReferenceFrame> fieldReference = new SendableChooser<FieldReferenceFrame>();

  private PIDController anglePID = new PIDController(1, 0, 0);

  private Translation2d targetPos = new Translation2d(0, 0);

  public ChassisSpeeds target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwerve(Drivetrain drivetrain, NavSensor nav, IControlInput controls) {
    _drivetrain = drivetrain;
    _controls = controls;
    addRequirements(drivetrain);

    fieldReference.addOption("Field", FieldReferenceFrame.Field);
    fieldReference.addOption("Robot", FieldReferenceFrame.Robot);
    fieldReference.setDefaultOption("Field", FieldReferenceFrame.Field);

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(fieldReference);
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
    _drivetrain.updatePoseFilter();
    Pose2d swerve_position = _drivetrain.poseFilter.getEstimatedPosition();

    m_field.setRobotPose(swerve_position);

    // Puts on shuffleboard
    SmartDashboard.putNumber("F rot", swerve_position.getRotation().getDegrees());
    SmartDashboard.putNumber("F X", swerve_position.getX());
    SmartDashboard.putNumber("F Y", swerve_position.getY());

    double speedLevel = _controls.getSpeedLevel();
    double speedMultiplier = speedLevel * (2 - 0.5) + 0.5;

    if (_controls.getResetPressed()) {
      _drivetrain.resetPose();
    }

    if (_controls.getPointPressed()) {
      targetPos = swerve_position.getTranslation();
    }

    double spin = anglePID.calculate(swerve_position.getRotation().getRadians(),
        AutoOrient.rotationFacingTarget(swerve_position.getTranslation(), targetPos).getRadians());
    if (!AutoOrient.shouldUpdateAngle(swerve_position.getTranslation(), targetPos)) {
      spin = 0;
    }

    _drivetrain.drive(
        _controls.getX() * speedMultiplier,
        _controls.getY() * speedMultiplier,
        !_controls.spinFlywheel() ? _controls.getSpin() : Math.max(-0.5, Math.min(0.5, spin)),
        fieldReference.getSelected() == FieldReferenceFrame.Field ? true : false);
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
