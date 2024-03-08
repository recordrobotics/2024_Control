package frc.robot.utils;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardChoosers {

    // Orient stuff
    public enum DriverOrientation {
        XAxisTowardsTrigger,
        YAxisTowardsTrigger
    }
    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();

    // Drivemodes
    public enum DriveMode {
        JoystickXbox, 
        DoubleXbox
    }
    private static SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();

    
    public ShuffleboardChoosers () {
        // Sets up joystick orientation
        driverOrientation.addOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        driverOrientation.addOption("Y Axis", DriverOrientation.YAxisTowardsTrigger);
        driverOrientation.setDefaultOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        SmartDashboard.putData("Joystick Orientation", driverOrientation);

        // Sets up shuffleboard
        driveMode.addOption("JoystickXbox", DriveMode.JoystickXbox);
        driveMode.addOption("DoubleXbox", DriveMode.DoubleXbox);
        driveMode.setDefaultOption("2Xbox", DriveMode.DoubleXbox);
        SmartDashboard.putData("Drive Mode", driveMode);
    }

    public static DriverOrientation getDriverOrientation() {
        return driverOrientation.getSelected();
    }

    public static DriveMode getDriveMode() {
        return driveMode.getSelected();
    }

}
