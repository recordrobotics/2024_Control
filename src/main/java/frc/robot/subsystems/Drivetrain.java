// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DriveCommandData;
import frc.robot.Constants;
import frc.robot.utils.DriverStationUtils;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

        // Creates Nav object
        private final NavSensor _nav = new NavSensor();

        // Creates swerve module objects
        private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.frontLeftConstants);
        private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.frontRightConstants);
        private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.backLeftConstants);
        private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.backRightConstants);

        // Creates swerve kinematics
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        Constants.Swerve.frontLeftConstants.wheelLocation,
                        Constants.Swerve.frontRightConstants.wheelLocation,
                        Constants.Swerve.backLeftConstants.wheelLocation,
                        Constants.Swerve.backRightConstants.wheelLocation);

        // Creates swerve post estimation filter
        public SwerveDrivePoseEstimator poseFilter;

        // Init drivetrain
        public Drivetrain() {
                _nav.relativeResetAngle();

                poseFilter = new SwerveDrivePoseEstimator(
                                m_kinematics,
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                },
                                DriverStationUtils.getCurrentAlliance() == Alliance.Red
                                                ? Constants.FieldConstants.TEAM_RED_STARTING_POSE
                                                : Constants.FieldConstants.TEAM_BLUE_STARTING_POSE);
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


        public void drive(DriveCommandData driveCommandData) {

                // Data from driveCommandData
                boolean fieldRelative = driveCommandData.fieldRelative;
                double xSpeed = driveCommandData.xSpeed;
                double ySpeed = driveCommandData.ySpeed;
                double rot = driveCommandData.rot;

                // Calculates swerveModuleStates given optimal ChassisSpeeds given by control scheme
                SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                poseFilter.getEstimatedPosition().getRotation())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

                // Desaturates wheel speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.robotMaxSpeed);

                // Sets state for each module
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
                                DriverStationUtils.getCurrentAlliance() == Alliance.Red
                                                ? Constants.FieldConstants.TEAM_RED_STARTING_POSE
                                                : Constants.FieldConstants.TEAM_BLUE_STARTING_POSE);
        }


}