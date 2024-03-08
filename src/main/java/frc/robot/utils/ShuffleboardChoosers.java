package frc.robot.utils;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.AbstractControl;

public class ShuffleboardChoosers {

    // Orient stuff
    public enum DriverOrientation {
        XAxisTowardsTrigger,
        YAxisTowardsTrigger
    }
    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();

    // Drive Modes
    private static SendableChooser<AbstractControl> driveMode = new SendableChooser<AbstractControl>();
    
    public static void initialize (AbstractControl defaultControl, AbstractControl... controls) {
        // Sets up joystick orientation
        driverOrientation.addOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        driverOrientation.addOption("Y Axis", DriverOrientation.YAxisTowardsTrigger);
        driverOrientation.setDefaultOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        SmartDashboard.putData("Joystick Orientation", driverOrientation);

        // Sets up shuffleboard
        driveMode.setDefaultOption(defaultControl.getClass().getName(), defaultControl);
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getName(), abstractControl);
        }
        SmartDashboard.putData("Drive Mode", driveMode);
    }

    public static DriverOrientation getDriverOrientation() {
        return driverOrientation.getSelected();
    }

    public static AbstractControl getDriveMode() {
        return driveMode.getSelected();
    }
}