// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class SwerveModule {

  // TODO: pretty sure this has to be 1 and 2, not pi and 2pi due to the way we
  // are inputting PID. Whatever, fix later
  // TODO: put in constants
  private static final double kModuleMaxAngularVelocity = 0.5; // Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 1; // 2 * Math.PI; // radians per second squared

  // Creates variables for motors and absolute encoders
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;
  private final double turningEncoderOffset;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      3,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute
   * turning encoder.
   *
   * @param driveMotorChannel
   * @param turningMotorChannel
   * @param absoluteTurningMotorEncoderChannel
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int absoluteTurningMotorEncoderChannel,
      double turningEncoderOffset) {

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    absoluteTurningMotorEncoder = new DutyCycleEncoder(absoluteTurningMotorEncoderChannel);
    this.turningEncoderOffset = turningEncoderOffset;

    // Limit the PID Controller's input range between -0.5 and 0.5 and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-0.5, 0.5);

    // Corrects for offset in absolute motor position
    m_turningMotor.setPosition(getAbsWheelOffset(absoluteTurningMotorEncoderChannel));

  }

  /**
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelOffset(int absoluteTurningMotorEncoderChannel) {
    double AbsEncoderPosition = (absoluteTurningMotorEncoder.getAbsolutePosition()
        - turningEncoderOffset + 1) % 1;
    double AbsWheelPositionoffset = -AbsEncoderPosition * Constants.Swerve.DIRECTION_GEAR_RATIO; // TODO: investigate
                                                                                                 // the "-" sign in this
                                                                                                 // line. I think it's
                                                                                                 // fine, but just to be
                                                                                                 // sure
    return AbsWheelPositionoffset;
  }

  /**
   * @return The current velocity of the drive motor (meters per second)
   */
  private double getDriveWheelVelocity() {
    double driveMotorRotationsPerSecond = m_driveMotor.getVelocity().getValue();
    double driveWheelMetersPerSecond = driveMotorRotationsPerSecond * 10 / Constants.Swerve.RELATIVE_ENCODER_RATIO
        * (Constants.Swerve.SWERVE_WHEEL_DIAMETER * Math.PI); // TODO: was originally 0.05 * 2 * pi. Also, why do we
                                                              // multiple by 10 here?
    return driveWheelMetersPerSecond;
  }

  /**
   * @return The raw rotations of the turning motor (rotation 2d object). NOT THE
   *         WHEEL. THE MOTOR.
   */
  private Rotation2d getTurnMotorRotation2d() {
    double numMotorRotations = m_turningMotor.getPosition().getValue();
    Rotation2d motorRotation = new Rotation2d(numMotorRotations * 2 * Math.PI / Constants.Swerve.DIRECTION_GEAR_RATIO);
    return motorRotation;
  }

  /**
   * @return The number of rotations of the turning wheel (rotations)
   */
  private double getTurnWheelRotations() {
    double numMotorRotations = m_turningMotor.getPosition().getValue();
    double numWheelRotations = numMotorRotations / Constants.Swerve.DIRECTION_GEAR_RATIO;
    return numWheelRotations;
  }

  /**
   * @return The distance driven by the drive wheel (meters)
   */
  private double getDriveWheelDistance() {
    double numRotationsMotor = m_driveMotor.getPosition().getValue();
    double numRotationsWheel = numRotationsMotor / Constants.Swerve.SPEED_GEAR_RATIO;
    double speedWheelDistanceMeters = numRotationsWheel * Math.PI * Constants.Swerve.SWERVE_WHEEL_DIAMETER;
    return speedWheelDistanceMeters;
  }

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        getDriveWheelVelocity(), getTurnMotorRotation2d());
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        getDriveWheelDistance(), getTurnMotorRotation2d());
  }

  /** resets drive motor position */
  public void resetDriveMotorPosition() {
    m_driveMotor.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getTurnMotorRotation2d());

    // Calculate the drive output from the drive PID controller then set drive
    // motor.
    final double driveOutput = m_drivePIDController.calculate(getDriveWheelVelocity(), state.speedMetersPerSecond);
    m_driveMotor.set(driveOutput);

    // Calculate the turning motor output from the turning PID controller then set
    // turn motor.
    final double turnOutput = m_turningPIDController.calculate(getTurnWheelRotations(), state.angle.getRotations());
    m_turningMotor.set(turnOutput);

  }
}