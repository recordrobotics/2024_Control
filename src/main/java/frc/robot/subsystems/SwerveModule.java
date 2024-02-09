// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  // TODO: put in constants
  private static final double kTurnMaxAngularVelocity = 5; // Drivetrain.kMaxAngularSpeed;
  private static final double kTurnMaxAngularAcceleration = 10; // 2 * Math.PI; // radians per second squared
  private static final double kMDriveMaxAngularVelocity = 10; // Drivetrain.kMaxAngularSpeed;
  private static final double kDriveMaxAngularAcceleration = 20; // 2 * Math.PI; // radians per second squared

  // Creates variables for motors and absolute encoders
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;
  private final double turningEncoderOffset;
  private final int driveMotorChannel;
  private final int turningMotorChannel;

  // Gains are for example purposes only - must be determined for your own robot!
  /*
   * private final PIDController m_drivePIDController = new PIDController(1, 0,
   * 0);
   **/

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      1,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kTurnMaxAngularVelocity, kTurnMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_drivePIDController = new ProfiledPIDController(
      3,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kMDriveMaxAngularVelocity, kDriveMaxAngularAcceleration));

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

    // Creates TalonFX objects
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    // Encoder number var creation
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;

    // Creates Motor Encoder object and gets offset
    absoluteTurningMotorEncoder = new DutyCycleEncoder(absoluteTurningMotorEncoderChannel);
    this.turningEncoderOffset = turningEncoderOffset;

    // 3 Seconds delay per swerve module
    Timer.delay(3);

    // Sets motor speeds to 0
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    // Limit the PID Controller's input range between -0.5 and 0.5 and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-0.5, 0.5);

    // Corrects for offset in absolute motor position
    m_turningMotor.setPosition(getAbsWheelTurnOffset());

    // SmartDashboard.putNumber("E" +
    // Double.toString(absoluteTurningMotorEncoderChannel),
    // absoluteTurningMotorEncoder.getAbsolutePosition());
  }

  /**
   * custom function
   * 
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelTurnOffset() {
    double absEncoderPosition = (absoluteTurningMotorEncoder.getAbsolutePosition() - turningEncoderOffset + 1) % 1;
    double absWheelPositionOffset = absEncoderPosition * Constants.Swerve.DIRECTION_GEAR_RATIO;
    return absWheelPositionOffset;
  }

  /**
   * TODO: figure out how this calculation works and make it more clear instead of
   * having it all happen on one line
   * custom function
   * 
   * @return The current velocity of the drive motor (meters per second)
   */
  private double getDriveWheelVelocity() {
    double driveMotorRotationsPerSecond = m_driveMotor.getVelocity().getValue();
    double driveWheelMetersPerSecond = driveMotorRotationsPerSecond * 10 / Constants.Swerve.RELATIVE_ENCODER_RATIO
        * (Constants.Swerve.SWERVE_WHEEL_DIAMETER * Math.PI);
    return driveWheelMetersPerSecond;
  }

  /**
   * custom function
   * 
   * @return The raw rotations of the turning motor (rotation 2d object). NOT THE
   *         WHEEL. THE MOTOR.
   */
  private Rotation2d getTurnWheelRotation2d() {
    double numMotorRotations = m_turningMotor.getPosition().getValue();
    Rotation2d motorRotation = new Rotation2d(numMotorRotations * 2 * Math.PI / Constants.Swerve.DIRECTION_GEAR_RATIO);
    return motorRotation;
  }

  /**
   * custom function
   * 
   * @return The number of rotations of the turning wheel (rotations)
   */
  private double getTurnWheelRotations() {
    double numMotorRotations = m_turningMotor.getPosition().getValue();
    double numWheelRotations = numMotorRotations / Constants.Swerve.DIRECTION_GEAR_RATIO;
    return numWheelRotations;
  }

  /**
   * custom function
   * 
   * @return The distance driven by the drive wheel (meters)
   */
  private double getDriveWheelDistance() {
    double numRotationsDriveMotor = m_driveMotor.getPosition().getValue();
    double numRotationsDriveWheel = numRotationsDriveMotor / Constants.Swerve.SPEED_GEAR_RATIO;
    double speedWheelDistanceMeters = numRotationsDriveWheel * Math.PI * Constants.Swerve.SWERVE_WHEEL_DIAMETER;
    return speedWheelDistanceMeters;
  }

  /**
   * custom function
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        getDriveWheelVelocity(), getTurnWheelRotation2d());
  }

  /**
   * custom function
   * 
   * @return The current position of the module as a SwerveModulePosition object.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        getDriveWheelDistance(), getTurnWheelRotation2d());
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

    // Puts temps on SmartDashboard
    SmartDashboard.putNumber("Temp turn " + turningMotorChannel, m_turningMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Temp drive " + driveMotorChannel, m_driveMotor.getDeviceTemp().getValueAsDouble());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,
        getTurnWheelRotation2d());

    // Calculate the drive output from the drive PID controller then set drive
    // motor.
    final double driveOutput = m_drivePIDController.calculate(getDriveWheelVelocity(),
        optimizedState.speedMetersPerSecond);
    m_driveMotor.set(driveOutput);

    // Calculate the turning motor output from the turning PID controller then set
    // turn motor.
    final double turnOutput = m_turningPIDController.calculate(getTurnWheelRotations(),
        optimizedState.angle.getRotations());
    m_turningMotor.set(turnOutput);

  }

}