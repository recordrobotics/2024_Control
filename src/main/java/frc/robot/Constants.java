// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
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

    public final class Shooter {
        public static final double SPEAKER_SPEED = 1;
        public static final double AMP_SPEED = 0.3;
        public static final double REVERSE_SPEED = -0.1;
    }

    public final class Channel {
        public static final double SHOOT_SPEED = 1;
        public static final double THROUGH_SPEED = 1;
        public static final double REVERSE_SPEED = -1;
    }

    public final class Acquisition {
        /** Constant; The speed and direction of the acquisition on intake */
        public static final double ACQUISITION_SPEED = 1;
    }

    public final class FieldConstants {

        public static final Translation2d TEAM_RED_SPEAKER = new Translation2d(16, 5.5);
        public static final Translation2d TEAM_BLUE_SPEAKER = new Translation2d(0.6, 5.6);
        public static final Rotation2d TEAM_RED_AMP = Rotation2d.fromDegrees(90); // new Translation2d(14.7, 8.5);
        public static final Rotation2d TEAM_BLUE_AMP = Rotation2d.fromDegrees(90); // new Translation2d(2.775, 8.5);

        // Field width and length
        public static final double FIELD_X_DIMENSION = 16.54; // Length
        public static final double FIELD_Y_DIMENSION = 8.21; // Width
    }

    public final class Control {

        // Sensitivity for speed meter
        public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
        public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.0;
        public static final double SPIN_SPEED_METER_LOW = 0.5;
        public static final double SPIN_SPEED_METER_HIGH = 4.3;

        // Sensitivies for directional controls (XY) and spin (theta)
        public static final double JOSYSTICK_DIRECTIONAL_SENSITIVITY = 1;
        public static final double JOYSTICK_SPIN_SENSITIVITY = 1;
        public static final double JOYSTICK_X_THRESHOLD = 0.15;
        public static final double JOYSTICK_Y_THRESHOLD = 0.15;
        public static final double JOYSTICK_SPIN_THRESHOLD = 0.76;

        // Thresholds for directional controls (XY) and spin (theta)
        public static final double XBOX_DIRECTIONAL_SENSITIVITY = 1;
        public static final double XBOX_X_THRESHOLD = 0.15;
        public static final double XBOX_Y_THRESHOLD = 0.15;
        public static final double XBOX_SPIN_THRESHOLD = 0.3;

        public static final double XBOX_SPIN_ROT_THRESHOLD = 0.1;
        public static final double XBOX_SPIN_ROT_SENSITIVITY = 1.0;

        // Tablet drive constants
        public final class Tablet {
            // Will fill in later, but for now it's convenient to have it in the TabletDrive
            public static final double PRESSURE_THRESHOLD = 0.2;
            public static final double MIN_SPEED = 0.2;
            public static final double STEEPNESS = 2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling
        }
    }

    public final class Swerve {

        // Works out module locations
        private static final double locX = Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        private static final double locY = Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
        public static final double locDist = Math.sqrt(locX * locX + locY * locY);

        private static final Translation2d frontLeftLocation = new Translation2d(locX, locY);
        private static final Translation2d frontRightLocation = new Translation2d(locX, -locY);
        private static final Translation2d backLeftLocation = new Translation2d(-locX, locY);
        private static final Translation2d backRightLocation = new Translation2d(-locX, -locY);

        // Gear ratios for falcon and kraken
        public static final double FALCON_TURN_GEAR_RATIO = 15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO = 7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

        public static final double KRAKEN_TURN_GEAR_RATIO = 13.3714;
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.75; // X1 12 pinion
        
        // PID Values
        public static final double FALCON_TURN_KP = 1;
        public static final double FALCON_TURN_KI = 0;
        public static final double FALCON_TURN_KD = 0;

        public static final double FALCON_DRIVE_KP = 0.2681;
        public static final double FALCON_DRIVE_KI = 0;
        public static final double FALCON_DRIVE_KD = 0;

        public static final double FALCON_DRIVE_FEEDFORWARD_KS = 0.1586;
        public static final double FALCON_DRIVE_FEEDFORWARD_KV = 2.4408;

        public static final double KRAKEN_TURN_KP = 2.3;
        public static final double KRAKEN_TURN_KI = 0;
        public static final double KRAKEN_TURN_KD = 0;

        public static final double KRAKEN_DRIVE_KP = 0.2681;
        public static final double KRAKEN_DRIVE_KI = 0;
        public static final double KRAKEN_DRIVE_KD = 0;

        public static final double KRAKEN_DRIVE_FEEDFORWARD_KS = 0.1205;
        public static final double KRAKEN_DRIVE_FEEDFORWARD_KV = 2.4915;

        // Same between Falcon and Kraken since they share the same encoders
        public static final double RELATIVE_ENCODER_RATIO = 2048;

        // Wheel diameter
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // Turn & Drive max velocity and acceleration
        public static final double TurnMaxAngularVelocity = 25; // Drivetrain.kMaxAngularSpeed;
        public static final double TurnMaxAngularAcceleration = 34; // 2 * Math.PI; // radians per second squared
        public static final double DriveMaxAngularVelocity = 15; // Drivetrain.kMaxAngularSpeed;
        public static final double DriveMaxAngularAcceleration = 30; // 2 * Math.PI; // radians per second squared

        /** The max speed the robot is allowed to travel */
        public static final double robotMaxSpeed = 7.0;

        public static final HolonomicPathFollowerConfig PathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
                3, // Max module speed, in m/s
                locDist, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        // Module Creation
        public static final ModuleConstants frontLeftConstants = new ModuleConstants(
                2,
                1,
                1,
                0.620, // 0.12363, //0.597
                frontLeftLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants frontRightConstants = new ModuleConstants(
                4,
                3,
                2,
                0.924, // 0.41599, //0.886
                frontRightLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants backLeftConstants = new ModuleConstants(
                8,
                7,
                4,
                0.676, // 0.17245, //0.857
                backLeftLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants backRightConstants = new ModuleConstants(
                6,
                5,
                3,
                0.371, // 0.86499, //0.556
                backRightLocation,
                MotorType.Falcon,
                MotorType.Falcon);
    }

    public final class Frame {

        /**
         * Distance between wheels (width aka between left and right and length aka
         * between front and back).
         * Used for calculating wheel locations on the robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.46;
        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.46;
    }
}
