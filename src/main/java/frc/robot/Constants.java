// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public final class Control {
        /**
         * Joystick sensitivity
         */
        public static final double INPUT_SENSITIVITY = 0.3;

        /**
         * Joystick spin sensitivity
         */
        public static final double SPIN_INPUT_SENSITIVITY = 0.5;

        /**
         * Joystick X input absolute threshold
         */
        public static final double INPUT_X_THRESHOLD = 0.1;

        /**
         * Joystick Y input absolute threshold
         */
        public static final double INPUT_Y_THRESHOLD = 0.1;

        /**
         * Joystick spin input absolute threshold
         */
        public static final double INPUT_SPIN_THRESHOLD = 0.2;
    }

    public final class Swerve {
        public static final double SPEED_GEAR_RATIO = 1;
        public static final double DIRECTION_GEAR_RATIO = 15.43;
        public static final double DIRECTION_KP = 3;
        public static final double DIRECTION_KI = 0;
        public static final double DIRECTION_KD = 0;
        public static final double RELATIVE_ENCODER_RATIO = 2048;
        public static final int SWERVE_WHEEL_COUNT = 4;

        /**
         * Distance between wheels (width). Used for the locations of the wheels on the
         * robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.59;

        /**
         * Distance between wheels (length). Used for the locations of the wheels on the
         * robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.59;

        // Absolute motor limit value (0 to 1)
        // Default is 1.0
        public static final double MOTOR_LIMIT = 1.0; // 0.8;

        // Limits the motor speed (value) between -MOTOR_LIMIT and MOTOR_LIMIT
        public static double LimitMotor(double value) {
            return Math.max(-MOTOR_LIMIT, Math.min(MOTOR_LIMIT, value));
        }
    }
}
