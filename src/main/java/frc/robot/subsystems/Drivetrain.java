// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

        // TODO: Move the below values to Constants
        public static final double kMaxSpeed = 3.0; // 3 meters per second
        public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

        private static final double wheelLocX = Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        private static final double wheelLocY = Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2;

        private final Translation2d m_frontLeftLocation  = new Translation2d(wheelLocX, wheelLocY);
        private final Translation2d m_frontRightLocation = new Translation2d(wheelLocX, -wheelLocY);
        private final Translation2d m_backLeftLocation   = new Translation2d(-wheelLocX, wheelLocY);
        private final Translation2d m_backRightLocation  = new Translation2d(-wheelLocX, -wheelLocY);

    // TODO: make sure the encoder values actually follow: front left, front right,
    // back left, back right
    private final SwerveModule m_frontLeft  = new SwerveModule(2, 1, 2, 0.628); //.411
    private final SwerveModule m_frontRight = new SwerveModule(4, 3, 3, 0.917); //.125
    private final SwerveModule m_backLeft   = new SwerveModule(8, 7, 5, 0.697); //.876
    private final SwerveModule m_backRight  = new SwerveModule(6, 5, 4, 0.363); //.193

    private final NavSensor _nav = new NavSensor();

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        // Creates swerve post estimation filter
        public SwerveDrivePoseEstimator poseFilter;

        public Drivetrain() {
                _nav.relativeResetAngle();

                // gives poseFilter value
                // TODO: currently using default standard deviations, find actual values.
                // TODO: currently sets starting pose to 0, 0. Input starting position once you
                // begin accounting for thems
                poseFilter = new SwerveDrivePoseEstimator(
                                m_kinematics,
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                },
                                new Pose2d(0, 0, new Rotation2d(0)));
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

                // Calculates swerveModuleStates given optimal ChassisSpeeds given by control scheme
                SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                poseFilter.getEstimatedPosition().getRotation())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

                // Desaturates wheel speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

                // Adds rotations of each module to SmartDashboards
                SmartDashboard.putNumber("r0", swerveModuleStates[0].angle.getDegrees());
                SmartDashboard.putNumber("r1", swerveModuleStates[1].angle.getDegrees());
                SmartDashboard.putNumber("r2", swerveModuleStates[2].angle.getDegrees());
                SmartDashboard.putNumber("r3", swerveModuleStates[3].angle.getDegrees());

                // Sets each module to desired state
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);
        }

        /** Updates the field relative position of the robot. */
        public void updatePoseFilter() {
                poseFilter.update(
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                });
        }

        /** Resets the field relative position of the robot (mostly for testing). */
        public void resetPose() {
                _nav.relativeResetAngle();
                m_frontLeft.resetDriveMotorPosition();
                m_frontRight.resetDriveMotorPosition();
                m_backLeft.resetDriveMotorPosition();
                m_backRight.resetDriveMotorPosition();
                poseFilter.resetPosition(
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                },
                                new Pose2d(0, 0, new Rotation2d(0)));
        }

}