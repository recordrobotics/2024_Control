// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  // TODO change to correct motor
  private TalonFX[] speedMotors = {
      new TalonFX(RobotMap.swerve.SPEED_MOTORS[0]),
      new TalonFX(RobotMap.swerve.SPEED_MOTORS[1]),
      new TalonFX(RobotMap.swerve.SPEED_MOTORS[2]),
      new TalonFX(RobotMap.swerve.SPEED_MOTORS[3])
  };

  private TalonFX[] directionMotors = {
      new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[0]),
      new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[1]),
      new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[2]),
      new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[3])
  };

  private CANCoder[] encoders = {
      new CANCoder(RobotMap.swerve.ENCODERS[0]),
      new CANCoder(RobotMap.swerve.ENCODERS[1]),
      new CANCoder(RobotMap.swerve.ENCODERS[2]),
      new CANCoder(RobotMap.swerve.ENCODERS[3])
  };

  private PIDController[] cANConPid = {
      new PIDController(RobotMap.swerve.PID.P[0], RobotMap.swerve.PID.I[0], RobotMap.swerve.PID.D[0]),
      new PIDController(RobotMap.swerve.PID.P[1], RobotMap.swerve.PID.I[1], RobotMap.swerve.PID.D[1]),
      new PIDController(RobotMap.swerve.PID.P[2], RobotMap.swerve.PID.I[2], RobotMap.swerve.PID.D[2]),
      new PIDController(RobotMap.swerve.PID.P[3], RobotMap.swerve.PID.I[3], RobotMap.swerve.PID.D[3])
  };

  Translation2d[] locations = {
      new Translation2d(Constants.Swerve.MOD_WIDTH / 2, Constants.Swerve.MOD_LENGTH / 2),
      new Translation2d(Constants.Swerve.MOD_WIDTH / 2, -(Constants.Swerve.MOD_LENGTH / 2)),
      new Translation2d(-(Constants.Swerve.MOD_WIDTH / 2), Constants.Swerve.MOD_LENGTH / 2),
      new Translation2d(-(Constants.Swerve.MOD_WIDTH / 2), -(Constants.Swerve.MOD_LENGTH / 2)),
  };

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations[0], locations[1],
      locations[2], locations[3]);

  private SwerveModuleState[] MOD_TARGETS;

  public AHRS _nav = new AHRS(SerialPort.Port.kUSB1);
  private static final SwerveModulePosition[] startPos = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics, _nav.getRotation2d(), startPos);

  SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      _nav.getRotation2d(),
      startPos,
      new Pose2d());

  ChassisSpeeds target;

  public Swerve() {
    speedMotors[0].set(ControlMode.Velocity, 0);
    speedMotors[1].set(ControlMode.Velocity, 0);
    speedMotors[2].set(ControlMode.Velocity, 0);
    speedMotors[3].set(ControlMode.Velocity, 0);

    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);
    directionMotors[0].set(ControlMode.Position, 0);

  }

  // gets current module states
  public SwerveModuleState[] modState() {
    SwerveModuleState[] state = {
        new SwerveModuleState(speedMotors[0].getSelectedSensorVelocity() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[0].getPosition() * 2 * Math.PI)),
        new SwerveModuleState(speedMotors[1].getSelectedSensorVelocity() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[0].getPosition() * 2 * Math.PI)),
        new SwerveModuleState(speedMotors[2].getSelectedSensorVelocity() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[0].getPosition() * 2 * Math.PI)),
        new SwerveModuleState(speedMotors[3].getSelectedSensorVelocity(),
            new Rotation2d(encoders[0].getPosition() * 2 * Math.PI))
    };
    return state;
  }

  public SwerveModulePosition[] modPos() {
    SwerveModulePosition[] pos = {
        new SwerveModulePosition(speedMotors[0].getSelectedSensorPosition() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[0].getPosition() * 2 * Math.PI)),
        new SwerveModulePosition(speedMotors[1].getSelectedSensorPosition() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[1].getPosition() * 2 * Math.PI)),
        new SwerveModulePosition(speedMotors[2].getSelectedSensorPosition() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[2].getPosition() * 2 * Math.PI)),
        new SwerveModulePosition(speedMotors[3].getSelectedSensorPosition() * Constants.Swerve.GEAR_RATIO,
            new Rotation2d(encoders[3].getPosition() * 2 * Math.PI))
    };
    return pos;
  }

  public void setTarget(ChassisSpeeds _target) {
    target = _target;
  }

  @Override
  public void periodic() {
    MOD_TARGETS = kinematics.toSwerveModuleStates(target);

    // optimises angle change
    SwerveModuleState.optimize(MOD_TARGETS[0], modState()[0].angle);
    SwerveModuleState.optimize(MOD_TARGETS[1], modState()[1].angle);
    SwerveModuleState.optimize(MOD_TARGETS[2], modState()[2].angle);
    SwerveModuleState.optimize(MOD_TARGETS[3], modState()[3].angle);

    // PID

    cANConPid[0].setSetpoint(MOD_TARGETS[0].angle.getRadians());
    cANConPid[1].setSetpoint(MOD_TARGETS[1].angle.getRadians());
    cANConPid[2].setSetpoint(MOD_TARGETS[2].angle.getRadians());
    cANConPid[3].setSetpoint(MOD_TARGETS[3].angle.getRadians());

    // sets speed/position of the motors
    speedMotors[0].set(ControlMode.Velocity, MOD_TARGETS[0].speedMetersPerSecond);
    speedMotors[1].set(ControlMode.Velocity, MOD_TARGETS[1].speedMetersPerSecond);
    speedMotors[2].set(ControlMode.Velocity, MOD_TARGETS[2].speedMetersPerSecond);
    speedMotors[3].set(ControlMode.Velocity, MOD_TARGETS[3].speedMetersPerSecond);

    directionMotors[0].set(ControlMode.PercentOutput, cANConPid[0].calculate(modState()[0].angle.getRadians()));
    directionMotors[1].set(ControlMode.PercentOutput, cANConPid[1].calculate(modState()[1].angle.getRadians()));
    directionMotors[2].set(ControlMode.PercentOutput, cANConPid[2].calculate(modState()[2].angle.getRadians()));
    directionMotors[3].set(ControlMode.PercentOutput, cANConPid[3].calculate(modState()[3].angle.getRadians()));

    m_PoseEstimator.update(_nav.getRotation2d(), modPos());
  }

  @Override
  public void simulationPeriodic() {

  }
}
