package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;

public class ModuleConstants {

    public int driveMotorChannel;
    public int turningMotorChannel;
    public int absoluteTurningMotorEncoderChannel;
    public double turningEncoderOffset;

    public Translation2d wheelLocation; //TODO: wheel location may be motor type dependent

    public double TURN_GEAR_RATIO;
    public double DRIVE_GEAR_RATIO;

    public double TURN_KP;
    public double TURN_KI;
    public double TURN_KD;

    public double DRIVE_KP;
    public double DRIVE_KI;
    public double DRIVE_KD;

    public double TurnMaxAngularVelocity;
    public double TurnMaxAngularAcceleration; 
    
    public double DriveMaxAngularVelocity;
    public double DriveMaxAngularAcceleration;

    public double RELATIVE_ENCODER_RATIO;
    public double MOTOR_LIMIT;
    public double WHEEL_DIAMETER;

    public ModuleConstants (

        int driveMotorChannel,
        int turningMotorChannel,
        int absoluteTurningMotorEncoderChannel,
        double turningEncoderOffset,
        Translation2d wheelLocation,

        //TODO: FINISH

        Object motorType) {

            this.driveMotorChannel = driveMotorChannel;
            this.turningMotorChannel = turningMotorChannel;
            this.absoluteTurningMotorEncoderChannel = absoluteTurningMotorEncoderChannel;
            this.turningEncoderOffset = turningEncoderOffset;
            this.wheelLocation = wheelLocation;

            if (motorType == new Kraken()) {
                this.TURN_KP = Constants.Swerve.KRAKEN_TURN_KD;
                this.TURN_KI = Constants.Swerve.KRAKEN_TURN_KI;
                this.TURN_KD = Constants.Swerve.KRAKEN_TURN_KD;

                this.DRIVE_KP = Constants.Swerve.KRAKEN_DRIVE_KD;
                this.DRIVE_KI = Constants.Swerve.KRAKEN_DRIVE_KI;
                this.DRIVE_KD = Constants.Swerve.KRAKEN_DRIVE_KD;

                this.TURN_GEAR_RATIO = Constants.Swerve.KRAKEN_TURN_GEAR_RATIO;
                this.DRIVE_GEAR_RATIO = Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO;
            }

            else {
                this.TURN_KP = Constants.Swerve.KRAKEN_TURN_KD;
                this.TURN_KI = Constants.Swerve.KRAKEN_TURN_KI;
                this.TURN_KD = Constants.Swerve.KRAKEN_TURN_KD;

                this.DRIVE_KP = Constants.Swerve.KRAKEN_DRIVE_KD;
                this.DRIVE_KI = Constants.Swerve.KRAKEN_DRIVE_KI;
                this.DRIVE_KD = Constants.Swerve.KRAKEN_DRIVE_KD;

                this.TURN_GEAR_RATIO = Constants.Swerve.KRAKEN_TURN_GEAR_RATIO;
                this.DRIVE_GEAR_RATIO = Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO;
            }


    }

    public class Falcon {}
    public class Kraken {}

}
