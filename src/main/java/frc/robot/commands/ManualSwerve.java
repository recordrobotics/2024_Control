// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavSensor;
import frc.robot.utils.AutoOrient;
import frc.robot.utils.DriverStationUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private DoubleControl _controls;

  private Field2d m_field = new Field2d();
  private SendableChooser<FieldReferenceFrame> fieldReference = new SendableChooser<FieldReferenceFrame>();

  private PIDController anglePID = new PIDController(0.4, 0, 0);

  private Translation2d targetPos = new Translation2d(0, 0);

  public ChassisSpeeds target;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwerve(Drivetrain drivetrain, NavSensor nav, DoubleControl controls) {
    _drivetrain = drivetrain;
    _controls = controls;
    addRequirements(drivetrain);

    anglePID.enableContinuousInput(-Math.PI, Math.PI);

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

    // Gets swerve position and sets to field position
    _drivetrain.updatePoseFilter();
    Pose2d swerve_position = _drivetrain.poseFilter.getEstimatedPosition();
    m_field.setRobotPose(swerve_position);

    // Puts on shuffleboard
    SmartDashboard.putNumber("F rot", swerve_position.getRotation().getDegrees());
    SmartDashboard.putNumber("F X", swerve_position.getX());
    SmartDashboard.putNumber("F Y", swerve_position.getY());

    // Gets speed level from controller
    double speedLevel = _controls.getSpeedLevel();
    double speedMultiplier = speedLevel * (2 - 0.5) + 0.5;

    // Control to reset pose if reset button is pressed
    if (_controls.getResetPressed()) {
      _drivetrain.resetPose();
    }

    boolean autoOrient = _controls.getAutoChain() || _controls.getAutoOrientSpeaker() || _controls.getAutoOrientAmp();

    if (_controls.getAutoOrientAmp()) {
      targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
          ? Constants.FieldConstants.TEAM_RED_AMP
          : Constants.FieldConstants.TEAM_BLUE_AMP;
    } else if (_controls.getAutoOrientSpeaker()) {
      targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
          ? Constants.FieldConstants.TEAM_RED_SPEAKER
          : Constants.FieldConstants.TEAM_BLUE_SPEAKER;
    }

    if (_controls.getKillAuto()) {
      // stop auto orient
      autoOrient = false;
    }

    double spin;
    if (autoOrient && AutoOrient.shouldUpdateAngle(swerve_position.getTranslation(), targetPos)) {
      spin = Math.max(-0.5, Math.min(0.5, anglePID.calculate(swerve_position.getRotation().getRadians(),
          AutoOrient.rotationFacingTarget(swerve_position.getTranslation(), targetPos).getRadians())));
    } else if (autoOrient) {
      spin = 0;
    } else {
      spin = _controls.getSpin();
    }

    // Drive command
    _drivetrain.drive(
        _controls.getX() * speedMultiplier,
        _controls.getY() * speedMultiplier,
        spin,
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
