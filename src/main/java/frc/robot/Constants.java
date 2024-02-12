// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class FieldConstants {

        public static final Translation2d TEAM_RED_SPEAKER = new Translation2d(16, 5.5);
        public static final Translation2d TEAM_BLUE_SPEAKER = new Translation2d(0.6, 5.6);
        public static final Translation2d TEAM_RED_AMP = new Translation2d(14.7, 8.5);
        public static final Translation2d TEAM_BLUE_AMP = new Translation2d(1.88, 8.5);

        // Field width and length
        //TODO: Remove testing values
        //public static final double FIELD_X_DIMENSION = 16.54; // Length
        //public static final double FIELD_Y_DIMENSION = 8.21; // Width
        public static final double FIELD_X_DIMENSION = 4.5; // Length
        public static final double FIELD_Y_DIMENSION = 3; // Width

        public static final Pose2d TEAM_RED_STARTING_POSE = new Pose2d(16, 4.2, Rotation2d.fromDegrees(180));
        //TODO: Remove testing values
        //public static final Pose2d TEAM_BLUE_STARTING_POSE = new Pose2d(0.5, 4.2, Rotation2d.fromDegrees(0));
        public static final Pose2d TEAM_BLUE_STARTING_POSE = new Pose2d(FIELD_X_DIMENSION/2, FIELD_Y_DIMENSION/2, Rotation2d.fromDegrees(0));
    }   

    public final class Control {
        /** Joystick sensitivity */
        public static final double INPUT_SENSITIVITY = 0.3;

        /** Joystick spin sensitivity */
        public static final double SPIN_INPUT_SENSITIVITY = 0.5;

        /** Joystick X input absolute threshold */
        public static final double INPUT_X_THRESHOLD = 0.15;

        /** Joystick Y input absolute threshold */
        public static final double INPUT_Y_THRESHOLD = 0.15;

        /** Joystick spin input absolute threshold */
        public static final double INPUT_SPIN_THRESHOLD = 0.5;

        // Tablet drive constants
        public final class Tablet {
            // Will fill in later, but for now it's convenient to have it in the TabletDrive file
        }
    }

    public final class Swerve {

        // Works out module locations
        private static final double locX = Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        private static final double locY = Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;

        private static final Translation2d frontLeftLocation = new Translation2d(locX, locY);
        private static final Translation2d frontRightLocation = new Translation2d(locX, -locY);
        private static final Translation2d backLeftLocation = new Translation2d(-locX, locY);
        private static final Translation2d backRightLocation = new Translation2d(-locX, -locY);

        // Module Creation
        public static final ModuleConstants frontLeftConstants = new ModuleConstants(
            2, 
            1, 
            2,
            0.628,
            frontLeftLocation, 
            MotorType.Falcon,
            MotorType.Falcon);
        public static final ModuleConstants frontRightConstants = new ModuleConstants(
            4, 
            3,
            3, 
            0.917, 
            frontRightLocation, 
            MotorType.Falcon, 
            MotorType.Falcon); 
        public static final ModuleConstants backLeftConstants = new ModuleConstants(
            8, 
            7, 
            5, 
            0.697, 
            backLeftLocation, 
            MotorType.Falcon, 
            MotorType.Falcon);
        public static final ModuleConstants backRightConstants = new ModuleConstants(
            6, 
            5, 
            4, 
            0.363, 
            backRightLocation, 
            MotorType.Falcon, 
            MotorType.Falcon);

        // Gear ratios for falcon and kraken
        public static final double KRAKEN_TURN_GEAR_RATIO = 7.85; //TODO: make sure these values are good
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.54;

        public static final double FALCON_TURN_GEAR_RATIO = 15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO = 7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        
        // PID Values
        public static final double FALCON_TURN_KP = 1;
        public static final double FALCON_TURN_KI = 0;
        public static final double FALCON_TURN_KD = 0;

        public static final double FALCON_DRIVE_KP = 3;
        public static final double FALCON_DRIVE_KI = 0;
        public static final double FALCON_DRIVE_KD = 0;

        public static final double KRAKEN_TURN_KP = 1;
        public static final double KRAKEN_TURN_KI = 0;
        public static final double KRAKEN_TURN_KD = 0;

        public static final double KRAKEN_DRIVE_KP = 3;
        public static final double KRAKEN_DRIVE_KI = 0;
        public static final double KRAKEN_DRIVE_KD = 0;

        // Shared
        public static final double RELATIVE_ENCODER_RATIO = 2048; // Same between Falcon and Kraken since they share the same encoders
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        public static final double TurnMaxAngularVelocity = 5; // Drivetrain.kMaxAngularSpeed;
        public static final double TurnMaxAngularAcceleration = 10; // 2 * Math.PI; // radians per second squared
        public static final double DriveMaxAngularVelocity = 10; // Drivetrain.kMaxAngularSpeed;
        public static final double DriveMaxAngularAcceleration = 20; // 2 * Math.PI; // radians per second squared

        /** The max speed the robot is allowed to travel */
        public static final double robotMaxSpeed = 3.0;
    }

    public final class Frame {

        /**Distance between wheels (width aka between left and right). 
         * Used for calculating wheel locations on the robot */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.59;

        /**Distance between wheels (length aka between front and back). 
         * Used for calculating wheel locations on the robot */
        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.59;
    }
}
