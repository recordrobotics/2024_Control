// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Shooter extends KillableSubsystem {

  private CANSparkMax flywheelL =
      new CANSparkMax( // initialize left flywheel
          RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID, MotorType.kBrushless);
  private CANSparkMax flywheelR =
      new CANSparkMax( // initialize right flywheel
          RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID, MotorType.kBrushless);

  private PIDController leftPID = new PIDController(0.07, 0, 0);
  private PIDController rightPID = new PIDController(0.07, 0, 0);

  private SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(0.12, 0.14);
  private SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(0.12, 0.13);

  private double targetVelocityLeft = 0.0;
  private double targetVelocityRight = 0.0;

  public Shooter() {
    toggle(ShooterStates.OFF); // initialize as off
    ShuffleboardUI.Test.addSlider(
            "Flywheel Left",
            flywheelL.get(),
            -1,
            1) // LEFT set slider to show value between -1 and 1
        .subscribe(flywheelL::set); // LEFT if the slider is moved, call flywheelL.set
    ShuffleboardUI.Test.addSlider(
            "Flywheel Right",
            flywheelR.get(),
            -1,
            1) // RIGHT set slider to show value between -1 and 1
        .subscribe(flywheelR::set); // RIGHT if the slider is moved, call flywheelR.set
  }

  public enum ShooterStates {
    SPEAKER, // High speed for speaker
    AMP, // Low speed for amp
    REVERSE, // Reverse for note acquisition
    OFF; // Off
  }

  public double getLeftWheelVelocity(){
    return flywheelL.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }

  public double getRightWheelVelocity(){
    return flywheelR.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }

  /** Set the current shooter speed to speedL and speedR */
  public void toggle(double speedL, double speedR) {
    targetVelocityLeft = -speedL; // left side is inverted because it is facing the other way
    targetVelocityRight = speedR;
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    toggle(speed, speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(ShooterStates state) {
    switch (state) {
      case SPEAKER: // High speed for speaker
        toggle(Constants.Shooter.SPEAKER_SPEED);
        break;
      case AMP: // Low speed for amp
        toggle(Constants.Shooter.AMP_SPEED);
        break;
      case REVERSE: // Reverse for note acquisition
        toggle(Constants.Shooter.REVERSE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0); // set speed to 0
        break;
    }
  }

  @Override
  public void periodic() {
    double leftPIDOutput =
        leftPID.calculate(getLeftWheelVelocity(), targetVelocityLeft);
    double leftFeedforwardOutput = leftFeedForward.calculate(targetVelocityLeft);
    flywheelL.setVoltage(leftPIDOutput + leftFeedforwardOutput); // Feed forward runs on voltage control

    double rightPIDOutput =
        rightPID.calculate(getRightWheelVelocity(), targetVelocityRight);
    double rightFeedforwardOutput = rightFeedForward.calculate(targetVelocityRight);
    flywheelR.setVoltage(
        rightPIDOutput + rightFeedforwardOutput); // Feed forward runs on voltage control

    ShuffleboardUI.Overview.putShooterSpeedData(
      0, getLeftWheelVelocity(), targetVelocityLeft);
    ShuffleboardUI.Overview.putShooterSpeedData(
      1, getRightWheelVelocity(), targetVelocityRight);
  }

  @Override
  public void kill() {
    toggle(ShooterStates.OFF);
    flywheelL.setVoltage(0);
    flywheelR.setVoltage(0);
  }
}
