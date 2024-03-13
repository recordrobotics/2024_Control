package frc.robot.utils;
import java.util.EnumSet;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ShuffleboardUI;
import frc.robot.control.AbstractControl;

public class ShuffleboardChoosers {

    // Orient stuff
    public enum DriverOrientation {
        XAxisTowardsTrigger,
        YAxisTowardsTrigger
    }
    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();

    public static SendableChooser<AutoName> autoChooser = new SendableChooser<>();

    // Drive Modes
    private static SendableChooser<AbstractControl> driveMode = new SendableChooser<AbstractControl>();
    private static AbstractControl _defaultControl;

    public enum AutoName{
        Speaker_2_note("2 Note Speaker"),
        Speaker_4_note("4 Note Speaker"),
        Amp_Speaker_2_note("Amp and Speaker (2)"),
        Amp_Speaker_1_note("Amp and Speaker"),
        Amp("Amp"),
        FarSpeaker("Far Speaker"),
        DiagLeftOneNote("DiagLeftOneNote"),
        DiagJustShoot("DiagJustShoot"),
        Speaker_3_Note("3NoteSpeaker"),
        Nutron1("Nutron1")
        ;
    
        private String pathref;
    
        public String getPathRef(){
          return pathref;
        }
    
        private AutoName(String pathplannerRef){
            pathref=pathplannerRef;
        }
      }

    /**
     * Initializes the control object
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public static void initialize (AbstractControl defaultControl, AbstractControl... controls) {
        _defaultControl = defaultControl;

        // Sets up joystick orientation
        driverOrientation.addOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        driverOrientation.addOption("Y Axis", DriverOrientation.YAxisTowardsTrigger);
        driverOrientation.setDefaultOption("X Axis", DriverOrientation.XAxisTowardsTrigger);
        
        var driverOrientationWidget = ShuffleboardUI.Overview.getTab()
            .add("Driver Orientation", driverOrientation);
        driverOrientationWidget.withWidget(BuiltInWidgets.kSplitButtonChooser);
        driverOrientationWidget.withPosition(0, 0);
        driverOrientationWidget.withSize(2, 1);

        // Sets up shuffleboard
        driveMode.setDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }

        var driveModeWidget = ShuffleboardUI.Overview.getTab()
            .add("Drive Mode", driveMode);
        driveModeWidget.withWidget(BuiltInWidgets.kSplitButtonChooser);
        driveModeWidget.withPosition(0, 1);
        driveModeWidget.withSize(3, 1);


        // Deals with shuffleboard chooser TODO: move to somewhere else
        EnumSet.allOf(AutoName.class)
            .forEach(v -> autoChooser.addOption(v.pathref, v));
        autoChooser.setDefaultOption(AutoName.Speaker_2_note.pathref, AutoName.Speaker_2_note);

        ShuffleboardUI.Autonomous.getTab()
        .add("Auto Code", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(3, 1)
        .withPosition(6, 1);
    }

    public static DriverOrientation getDriverOrientation() {
        return driverOrientation.getSelected();
    }

    public static AbstractControl getDriveControl() {
        if(driveMode.getSelected() == null)
            return _defaultControl;
        return driveMode.getSelected();
    }
}