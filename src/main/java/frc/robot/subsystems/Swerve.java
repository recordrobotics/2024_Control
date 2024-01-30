// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

        /**
         * Offsets for Absolute encoder
         */
        private final double ENCODER_OFFSETS[] = { 0.411, 0.126, 0.864, 0.194 };

        /**
         * The number of wheels on the robot
         */
        private final int wheelCount = Constants.Swerve.SWERVE_WHEEL_COUNT;

        /**
         * Motors that spin the wheels to move the robot
         */
        private TalonFX[] speedMotors = new TalonFX[wheelCount];

        /**
         * Motors that rotate the wheels to change the direction of movement of the
         * robot
         */
        private TalonFX[] directionMotors = new TalonFX[wheelCount];

        /**
         * Encoders for initializing the direction motor's position
         */
        private DutyCycleEncoder[] encoders = new DutyCycleEncoder[wheelCount];

        /**
         * Direction motor PIDs
         */
        private PIDController[] directionPID = new PIDController[wheelCount];

        /**
         * Target swerve module states that are updated and optimized in periodic
         */
        private SwerveModuleState[] targetStates = new SwerveModuleState[wheelCount];

        /**
         * Locations of the wheels on the robot frame.
         */
        Translation2d[] wheelLocations = {
                        new Translation2d(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2,
                                        Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2),
                        new Translation2d(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2,
                                        -(Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2)),
                        new Translation2d(-(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2),
                                        Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2),
                        new Translation2d(-(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2),
                                        -(Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2)),
        };

        /**
         * Kinematics for the swerve drive with the four wheel locations. Used in
         * periodic to calculate the target wheel states.
         */
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        wheelLocations[0],
                        wheelLocations[1],
                        wheelLocations[2],
                        wheelLocations[3]);

        /**
         * Target Velocity and Angle of the chassis
         */
        ChassisSpeeds targetChassisSpeed = new ChassisSpeeds();

        // Nav
        private NavSensor _nav = new NavSensor();

        // Creates swerve post estimation filter
        public SwerveDrivePoseEstimator poseFilter;
        // accurate values

        public Swerve() {
                // Create motor objects
                for (int i = 0; i < wheelCount; i++) {
                        // Create new TalonFX objects for each speed and direction motor
                        speedMotors[i] = new TalonFX(RobotMap.swerve.SPEED_MOTOR_DEVICE_IDS[i]);
                        directionMotors[i] = new TalonFX(RobotMap.swerve.DIRECTION_MOTOR_DEVICE_IDS[i]);
                        // Create new DutyCycleEncoder objects for each encoder
                        encoders[i] = new DutyCycleEncoder(RobotMap.swerve.ENCODER_DEVICE_IDS[i]);
                        // Create new PIDController objects for each direction motor
                        directionPID[i] = new PIDController(Constants.Swerve.DIRECTION_KP,
                                        Constants.Swerve.DIRECTION_KI,
                                        Constants.Swerve.DIRECTION_KD);
                        // Create new SwerveModuleState objects for each wheel
                        targetStates[i] = new SwerveModuleState();
                }

                // Worried about latency in reading of some values so wait a few seconds
                Timer.delay(5);

                // Init the motor and PID values
                for (int i = 0; i < wheelCount; i++) {
                        SmartDashboard.putNumber("Init Abs" + i, encoders[i].getAbsolutePosition());

                        // Reset motor speed
                        speedMotors[i].set(0);
                        directionMotors[i].set(0);

                        // Offset direction motor encoder position
                        final double encoderValue = getEncoderPosition(i);
                        final double encoderValueWithRatio = -encoderValue
                                        * Constants.Swerve.DIRECTION_GEAR_RATIO;
                        // Set direction motor position offset
                        directionMotors[i].setPosition(encoderValueWithRatio);
                        directionPID[i].enableContinuousInput(-0.5, 0.5);
                }

                // gives poseFilter value
                poseFilter = new SwerveDrivePoseEstimator(kinematics, _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                getPosition(0), getPosition(1), getPosition(2), getPosition(3) },
                                new Pose2d(0, 0, new Rotation2d(0)));// TODO: currently using default standard
                                                                     // deviations, get

        }

        /**
         * Gets the absolute encoder position offsetted using ENCODER_OFFSETS
         * 
         * @param encoderIndex index of encoder in array
         * @return the absolute position of the encoder relative to our our robots zero
         */
        private double getEncoderPosition(int encoderIndex) {
                return (encoders[encoderIndex].getAbsolutePosition() - ENCODER_OFFSETS[encoderIndex] + 1) % 1;
        }

        /**
         * Gets the direction motor position in rotations
         * 
         * @param motorId index of motor in array
         */
        private double getDirectionMotorRotations(int motorId) {
                return directionMotors[motorId].getPosition().getValue();
        }

        /**
         * Gets the direction of the wheel in rotations (motor rotation with gear ratio
         * taken into account)
         * 
         * @param motorId index of motor in array
         */
        private double getDirectionWheelRotations(int motorId) {
                double numRotations = getDirectionMotorRotations(motorId);
                return (numRotations)
                                / Constants.Swerve.DIRECTION_GEAR_RATIO;
        }

        /**
         * Gets the distance (in meters) travelled by the speed wheel
         * 
         * @param motorId index of motor in array
         */
        private double getSpeedWheelDistanceMeters(int motorId) {
                double numRotationsMotor = speedMotors[motorId].getPosition().getValue();
                double numRotationsWheel = numRotationsMotor / Constants.Swerve.SPEED_GEAR_RATIO;
                double speedWheelDistanceMeters = numRotationsWheel * Math.PI * Constants.Swerve.SWERVE_WHEEL_DIAMETER;

                return speedWheelDistanceMeters;
        }

        /**
         * Gets the current wheel states (speed and rotation in radians)
         */
        public SwerveModuleState[] getCurrentSwerveState() {
                SwerveModuleState[] states = new SwerveModuleState[wheelCount];
                for (int i = 0; i < wheelCount; i++) {
                        states[i] = new SwerveModuleState(
                                        // Current speed in meters per second
                                        speedMotors[i].getVelocity().getValue() * 10
                                                        / Constants.Swerve.RELATIVE_ENCODER_RATIO
                                                        * (0.05 * 2 * Math.PI),
                                        // Current rotation in radians
                                        new Rotation2d(
                                                        getDirectionMotorRotations(i)

                                                                        * 2 * Math.PI
                                                                        / Constants.Swerve.DIRECTION_GEAR_RATIO));
                }
                return states;
        }

        /**
         * Sets the target chassis speed
         */
        public void setTargetChassisSpeed(ChassisSpeeds _target) {
                targetChassisSpeed = _target;
        }

        // returns a SwerveModulePosition object for use in the SwerveDrivePoseEstimator
        public SwerveModulePosition getPosition(int i) {
                return new SwerveModulePosition(
                                // TODO: I have no idea if these values are the correct ones. See
                                // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot/SwerveModule.java
                                getSpeedWheelDistanceMeters(i),
                                new Rotation2d(getDirectionWheelRotations(i) * 2 * Math.PI));
        }

        public void resetPose() {
                _nav.relativeResetAngle();
                for (int i = 0; i < wheelCount; i++) {
                        speedMotors[i].setPosition(0);
                }
                poseFilter.resetPosition(_nav.getAdjustedAngle(), new SwerveModulePosition[] {
                                getPosition(0), getPosition(1), getPosition(2), getPosition(3) },
                                new Pose2d(0, 0, new Rotation2d(0)));
        }

        @Override
        public void periodic() {

                // Updates poseFilter
                poseFilter.update(_nav.getAdjustedAngle(), new SwerveModulePosition[] {
                                getPosition(0), getPosition(1), getPosition(2), getPosition(3) });

                // Run kinematics to convert target speeds to swerve wheel angle and rotations
                targetStates = kinematics.toSwerveModuleStates(targetChassisSpeed);
                // Get current swerve wheel states
                SwerveModuleState[] currentStates = getCurrentSwerveState();

                for (int i = 0; i < wheelCount; i++) {
                        // Optimize rotation and speed before using values
                        targetStates[i] = SwerveModuleState.optimize(targetStates[i], currentStates[i].angle);

                        SmartDashboard.putNumber("Abs Encoder " + i, encoders[i].getAbsolutePosition());
                        SmartDashboard.putNumber("Offset Abs Encoder" + i, getEncoderPosition(i));

                        // Set target wheel rotations for the PID
                        directionPID[i].setSetpoint(targetStates[i].angle.getRotations());

                        // Set speed motor speed (-1 to 1)
                        speedMotors[i].set(Constants.Swerve.LimitMotor(targetStates[i].speedMetersPerSecond));

                        // Set direction motor speed based on feedback from PID controller
                        double wheelRotations = getDirectionWheelRotations(i);
                        double dpidCalculation = directionPID[i].calculate(wheelRotations);
                        directionMotors[i].set(dpidCalculation);
                }

        }

        @Override
        public void simulationPeriodic() {
                periodic();
        }
}
