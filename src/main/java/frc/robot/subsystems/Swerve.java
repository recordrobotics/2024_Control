// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

//import com.ctre.phoenix6.motorcontrol.ControlMode;

//import com.ctre.phoenix6.

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {
        // TODO: Add commment explaining how you got these constants.
        private final double moduleWidth = 0.762;
        private final double moduleLength = 0.762;

        /**
         * Offsets for Absolute encoder
         * @apiNote
         * Almost certainly junk as they are for a different robot, keeping them as their current values for now but we need to delete them when we build our new frame
         */
        private final double ABS_AT_ZERO[] = {0.411, 0.126, 0.864, 0.194};

        private final int numMotors = Constants.Swerve.NUM_SWERVE_MODS;
        /*Turn Wheels*/
        private TalonFX[] speedMotors = new TalonFX[numMotors];
        /* Change the angle of the wheels*/ 
        private TalonFX[] directionMotors = new TalonFX[numMotors];
        private DutyCycleEncoder[] encoders = new DutyCycleEncoder[numMotors];
        
        private PIDController[] dPID = new PIDController[numMotors];
        private PIDController[] sPID = new PIDController[numMotors];

        private SwerveModuleState[] modTargets = new SwerveModuleState[numMotors];
        private AHRS _nav = new AHRS(SerialPort.Port.kUSB1);
        private double angle0;

        // TODO: Add commment explaining what this is doing.
        Translation2d[] locations = {
                        new Translation2d(moduleWidth / 2, moduleLength / 2),
                        new Translation2d(moduleWidth / 2, -(moduleLength / 2)),
                        new Translation2d(-(moduleWidth / 2), moduleLength / 2),
                        new Translation2d(-(moduleWidth / 2), -(moduleLength / 2)),
        };

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations[0], locations[1],
                        locations[2], locations[3]);

        /*
         * private static final SwerveModulePosition[] startPos = {
         * new SwerveModulePosition(),
         * new SwerveModulePosition(),
         * new SwerveModulePosition(),
         * new SwerveModulePosition()
         * };
         */
        // SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(kinematics,
        // _nav.getRotation2d(), startPos);

        /**
         * Target Velocity and Angle
         */
        ChassisSpeeds target = new ChassisSpeeds();

        public Swerve() {
                // Does this need to be calibrated?
                //_nav.calibrate();
                _nav.reset();
                _nav.resetDisplacement();
                angle0 = _nav.getAngle();
                SmartDashboard.putBoolean("Nav connected", _nav.isConnected());
                SmartDashboard.putBoolean("Nav Cal", _nav.isCalibrating());

                // Init Motors
                for (int i = 0; i < numMotors; i++) {
                        // motors
                        speedMotors[i] = new TalonFX(RobotMap.swerve.SPEED_MOTORS[i]);
                        directionMotors[i] = new TalonFX(RobotMap.swerve.DIRECTION_MOTORS[i]);
                        // absolute encoders
                        encoders[i] = new DutyCycleEncoder(RobotMap.swerve.DEVICE_NUMBER[i]);
                        // PID
                        dPID[i] = new PIDController(Constants.Swerve.kp, Constants.Swerve.ki, Constants.Swerve.kd);
                        sPID[i] = new PIDController(0, 0, 0);

                        modTargets[i] = new SwerveModuleState();
                }

                // TODO: Add commment explaining why this delay is here.
                Timer.delay(5);

                // motor + PID settings
                for (int i = 0; i < numMotors; i++) {
                        SmartDashboard.putNumber("Init Abs" + i, encoders[i].getAbsolutePosition());
                        //directionMotors[i].configNeutralDeadband(0.001);

                        speedMotors[i].set(0);
                        directionMotors[i].set( 0);
                        final double curAbsPos = getOffsetAbs(i);
                        final double curRelPos = -curAbsPos * Constants.Swerve.RELATIVE_ENCODER_RATIO
                                        * Constants.Swerve.DIRECTION_GEAR_RATIO;
                        directionMotors[i].setPosition(curRelPos);
                        dPID[i].enableContinuousInput(-0.5, 0.5);
                }

        }

        /**
         * 
         * @param encoderIndex index of encoder in array
         * @return the absolute position of the encoder relative to our our robots zero
         */
        private double getOffsetAbs(int encoderIndex) {
                return (encoders[encoderIndex].getAbsolutePosition() - ABS_AT_ZERO[encoderIndex] + 1) % 1;
        }

        public Rotation2d getAngle() {
                return new Rotation2d(-(_nav.getAngle() - angle0) / 180 * Math.PI);
        }

        /* 
        public double getCompassHeading() {
                System.out.println(_nav.getCompassHeading());
                return (double) _nav.getCompassHeading();
        }
        */

        private double getDirectionMotorRotations(int motorNum) {
                return directionMotors[motorNum].getRotorPosition().getValue();
        }

        /**
         * 
         * @param motorNum index of motor in array
         * @return Relative encoder in rotations
         */
        private double getRelInRotations(int motorNum) {
                double numRotations  = getDirectionMotorRotations(motorNum);
                return (numRotations / Constants.Swerve.RELATIVE_ENCODER_RATIO)
                                / Constants.Swerve.DIRECTION_GEAR_RATIO;
        }



        /**
         * gets current module states
         */
        public SwerveModuleState[] modState() {
                SwerveModuleState[] LFState = new SwerveModuleState[numMotors];
                for (int i = 0; i < numMotors; i++) {
                        LFState[i] = new SwerveModuleState(
                                        // TODO: Both of these parameters have a fair amount of math going on,
                                        // it would be better to factor them out and have clear comments
                                        // explaining what the conversion is doing.
                                        speedMotors[i].getVelocity().getValue() * 10
                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                        * (0.05 * 2 * Math.PI),
                                        new Rotation2d(
                                                        getDirectionMotorRotations(i)
                                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                                        * 2 * Math.PI
                                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                }
                return LFState;
        }

        /**
         * run in manualswerve to get target speed
         */
        public void setTarget(ChassisSpeeds _target) {
                target = _target;
        }

        /**
         * Resets robot field frame angle
         */
        public void resetAngle() {
                angle0 = _nav.getAngle();
        }

        @Override
        public void periodic() {
                SmartDashboard.putBoolean("Nav connected", _nav.isConnected());
                SmartDashboard.putBoolean("Nav Cal", _nav.isCalibrating());
                SmartDashboard.putNumber("getAngle()", (double) _nav.getAngle());
                //SmartDashboard.putNumber("Compass", getCompassHeading());
                // converts target speeds to swerve module angle and rotations
                modTargets = kinematics.toSwerveModuleStates(target);
                for (int i = 0; i < numMotors; i++) {
                        // Optimize before using values
                        modTargets[i] = SwerveModuleState.optimize(modTargets[i], new Rotation2d(
                                                        getDirectionMotorRotations(i)
                                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                                        * 2 * Math.PI
                                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                        
                        SmartDashboard.putNumber("Abs Encoder " + i, encoders[i].getAbsolutePosition());
                        SmartDashboard.putNumber("Offset Abs Encoder" + i, getOffsetAbs(i));
                        SmartDashboard.putNumber("M" + i, modTargets[i].angle.getRotations());
                        SmartDashboard.putNumber("Relative " + i, getRelInRotations(i));
                        // position PIDs
                        dPID[i].setSetpoint(modTargets[i].angle.getRotations());
                        sPID[i].setSetpoint(modTargets[i].speedMetersPerSecond);
                        // sets speed/position of the motors
                        speedMotors[i].set(sPID[i].calculate(speedMotors[i].getVelocity().getValue()
                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                        / Constants.Swerve.SPEED_GEAR_RATIO * 0.6383716272));
                        double simpleRelativeEncoderVal = getRelInRotations(i);
                        directionMotors[i].set(dPID[i].calculate(simpleRelativeEncoderVal));
                }
        }

        @Override
        public void simulationPeriodic() {
                periodic();
        }
}
