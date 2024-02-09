package frc.robot;

public final class RobotMap {

    public static final class Aquisition {
        public static final int lower = 0;
        public static final int upper = 0;
    }

    public static final class Climbers {
        public static final int left = 0;
        public static final int right = 0;
    }

    public static final class Shooter {
        public static final int flywheel = 0;
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
        public static final int STICKPAD_PORT = 0;
        public static final int GAMEPAD_PORT = 1;
    }
}
