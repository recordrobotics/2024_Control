// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.utils.ModuleConstantsObject;
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

    /**
     * Remaps value between min and max to 0 - 1
     * Works with negative values: remap(x, min, max) = -remap(-x, min, max)
     * 
     * @implNote NOTE! DOES NOT CLAMP OUTPUT TO 0 - 1 RANGE.
     * @param value  The value between absMin and absMax
     * @param absMin Min value of the input (always positive, even if input is
     *               negative)
     * @param absMax Max value of the input (always positive, even if input is
     *               negative)
     * @return Returns a value between 0 and 1 or 0 and -1 depending on the sign of
     *         the input value
     */
    public static double RemapAbsoluteValue(double value, double absMin, double absMax) {
        if (value >= absMin) { // positive relative value
            return (value - absMin) / (absMax - absMin);
        } else if (value <= -absMin) { // negative relative value
            return (value - absMin) / (absMax - absMin) + 2;
        } else {
            return 0;
        }
    }

    public final class FieldConstants {

        public static final Translation2d TEAM_RED_SPEAKER = new Translation2d(16, 5.5);
        public static final Translation2d TEAM_BLUE_SPEAKER = new Translation2d(0.6, 5.6);
        public static final Translation2d TEAM_RED_AMP = new Translation2d(14.7, 8.5);
        public static final Translation2d TEAM_BLUE_AMP = new Translation2d(1.88, 8.5);

        // Field width and length
        //public static final double FIELD_X_DIMENSION = 16.54; // Length
        //public static final double FIELD_Y_DIMENSION = 8.21; // Width
        public static final double FIELD_X_DIMENSION = 4.5; // Length
        public static final double FIELD_Y_DIMENSION = 3; // Width


        public static final Pose2d TEAM_RED_STARTING_POSE = new Pose2d(16, 4.2, Rotation2d.fromDegrees(180));
        //public static final Pose2d TEAM_BLUE_STARTING_POSE = new Pose2d(0.5, 4.2, Rotation2d.fromDegrees(0));
        public static final Pose2d TEAM_BLUE_STARTING_POSE = new Pose2d(FIELD_X_DIMENSION/2, FIELD_Y_DIMENSION/2, Rotation2d.fromDegrees(0));
    }   

    public final class Control {
        /** Joystick sensitivity */
        public static final double INPUT_SENSITIVITY = 0.3;

        /** Joystick spin sensitivity */
        public static final double SPIN_INPUT_SENSITIVITY = 0.5;

        /** Joystick spin remap low value (remaps LOW-HIGH to 0-1) */
        public static final double SPIN_INPUT_REMAP_LOW = 0.5;

        /** Joystick spin remap high value (remaps LOW-HIGH to 0-1) */
        public static final double SPIN_INPUT_REMAP_HIGH = 1;

        /** Joystick X input absolute threshold */
        public static final double INPUT_X_THRESHOLD = 0.15;

        /** Joystick Y input absolute threshold */
        public static final double INPUT_Y_THRESHOLD = 0.15;

        /** Joystick spin input absolute threshold */
        public static final double INPUT_SPIN_THRESHOLD = 0.5;
    }


    public final class Swerve {

        // Module Creation
        //public ModuleConstantsObject frontLeftModuleConstants = new ModuleConstantsObject();

        //public ModuleConstantsObject frontRightModuleConstants = 


        // Gear ratios for falcon and kraken
        public static final double KRAKEN_TURN_GEAR_RATIO = 1; //TODO: find these gear ratios
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 1;

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

        // Motor Limits
        /** Absolute motor limit value (0 to 1), default is 1.0 */
        public static final double FALCON_MOTOR_LIMIT = 1.0; // 0.8;
        public static final double KRAKEN_MOTOR_LIMIT = 1.0; // 0.8;

        // Shared
        public static final double RELATIVE_ENCODER_RATIO = 2048; // Same between Falcon and Kraken since they share the same encoders
        public static final double SWERVE_WHEEL_DIAMETER = Units.inchesToMeters(4);
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
