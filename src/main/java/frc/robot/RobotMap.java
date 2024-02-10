package frc.robot;

public final class RobotMap {

    public static final class Aquisition {
        public static final int LOWER_ACQUISITION_MOTOR_ID = -1;
        public static final int UPPER_ACQUISITION_MOTOR_ID = -1;

    }

    public static final class Climbers {
        public static final int LEFT_FORWARD_PORT = -1;
        public static final int LEFT_REVERSE_PORT = -1;
        public static final int RIGHT_FORWARD_PORT = -1;
        public static final int RIGHT_REVERSE_PORT = -1;
    }

    public static final class Shooter {
        public static final int flywheel = -1;
    }

    public static final class swerve {
        // todo change to correct ports
        public static final int SPEED_MOTOR_FRONT_LEFT_DEVICE_ID = 2;
        public static final int SPEED_MOTOR_FRONT_RIGHT_DEVICE_ID = 4;
        public static final int SPEED_MOTOR_BACK_LEFT_DEVICE_ID = 8;
        public static final int SPEED_MOTOR_BACK_RIGHT_DEVICE_ID = 6;

        public static final int DIRECTION_MOTOR_FRONT_LEFT_DEVICE_ID = 1;
        public static final int DIRECTION_MOTOR_FRONT_RIGHT_DEVICE_ID = 3;
        public static final int DIRECTION_MOTOR_BACK_LEFT_DEVICE_ID = 7;
        public static final int DIRECTION_MOTOR_BACK_RIGHT_DEVICE_ID = 5;

        public static final int ENCODER_FRONT_LEFT_DEVICE_ID = 2;
        public static final int ENCODER_FRONT_RIGHT_DEVICE_ID = 3;
        public static final int ENCODER_BACK_LEFT_DEVICE_ID = 5;
        public static final int ENCODER_BACK_RIGHT_DEVICE_ID = 4;
    }

    public static class Control {
        public static final int SINGLE_GAMEPAD = 0;
    }
}
