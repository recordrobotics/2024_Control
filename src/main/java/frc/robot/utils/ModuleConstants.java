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
    public double WHEEL_DIAMETER;

    /**
     * essentially serves as a storage unit for one swerve module, storing every single constant that a module might want to use
     * @param driveMotorChannel drive motor port
     * @param turningMotorChannel turn motor port
     * @param absoluteTurningMotorEncoderChannel abs turn motor encoder port
     * @param turningEncoderOffset offset of the abs turn encoder at a set starting position (which we found through manually testing)
     * @param wheelLocation Translation2d object of where the wheel is relative to robot frame
     * @param turnIsFalcon Whether or not the turn motor is a Falcon motor. If false, assumes that the turn motor is a Kraken
     * @param driveIsFalcon Whether or not the drive motor is a Falcon motor. If false, assumes that the drive motor is a Kraken
     */
    public ModuleConstants (

        int driveMotorChannel,
        int turningMotorChannel,
        int absoluteTurningMotorEncoderChannel,
        double turningEncoderOffset,
        Translation2d wheelLocation,

        Boolean turnIsFalcon,
        Boolean driveIsFalcon) {

            // Encoder nums
            this.driveMotorChannel = driveMotorChannel;
            this.turningMotorChannel = turningMotorChannel;
            this.absoluteTurningMotorEncoderChannel = absoluteTurningMotorEncoderChannel;
            this.turningEncoderOffset = turningEncoderOffset;

            // Wheel location
            this.wheelLocation = wheelLocation;
            
            // Max Angular Acceleration & Velocity
            this.TurnMaxAngularVelocity = Constants.Swerve.TurnMaxAngularVelocity;
            this.TurnMaxAngularAcceleration = Constants.Swerve.TurnMaxAngularAcceleration;
            this.DriveMaxAngularVelocity = Constants.Swerve.DriveMaxAngularVelocity;
            this.DriveMaxAngularAcceleration = Constants.Swerve.DriveMaxAngularAcceleration;

            // Shared miscellaneous variables
            this.RELATIVE_ENCODER_RATIO = Constants.Swerve.RELATIVE_ENCODER_RATIO;
            this.WHEEL_DIAMETER = Constants.Swerve.WHEEL_DIAMETER;

            // Turn Motor Constants
            if (turnIsFalcon) {
                this.TURN_KP = Constants.Swerve.FALCON_TURN_KD;
                this.TURN_KI = Constants.Swerve.FALCON_TURN_KI;
                this.TURN_KD = Constants.Swerve.FALCON_TURN_KD;
                this.TURN_GEAR_RATIO = Constants.Swerve.FALCON_TURN_GEAR_RATIO;
            }

            else {
                this.TURN_KP = Constants.Swerve.KRAKEN_TURN_KD;
                this.TURN_KI = Constants.Swerve.KRAKEN_TURN_KI;
                this.TURN_KD = Constants.Swerve.KRAKEN_TURN_KD;
                this.TURN_GEAR_RATIO = Constants.Swerve.KRAKEN_TURN_GEAR_RATIO;
            }

            // Drive Motor Constants
            if (driveIsFalcon) {
                this.DRIVE_KP = Constants.Swerve.FALCON_DRIVE_KD;
                this.DRIVE_KI = Constants.Swerve.FALCON_DRIVE_KI;
                this.DRIVE_KD = Constants.Swerve.FALCON_DRIVE_KD;
                this.DRIVE_GEAR_RATIO = Constants.Swerve.FALCON_DRIVE_GEAR_RATIO;
            }

            else {
                this.DRIVE_KP = Constants.Swerve.KRAKEN_DRIVE_KD;
                this.DRIVE_KI = Constants.Swerve.KRAKEN_DRIVE_KI;
                this.DRIVE_KD = Constants.Swerve.KRAKEN_DRIVE_KD;
                this.DRIVE_GEAR_RATIO = Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO;
            }
                

    }

}
