package frc.robot.utils;
import java.util.EnumSet;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.control.AbstractControl;

public class ShuffleboardChoosers {

    // Orient stuff
    public enum DriverOrientation {
        XAxisTowardsTrigger,
        YAxisTowardsTrigger
    }

    // Auto routines
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
    
        public final String pathref;
    
        AutoName(String pathplannerRef){
            pathref=pathplannerRef;
        }
      }
      
    
    // Sets up sendable choosers
    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();
    public static SendableChooser<AutoName> autoChooser = new SendableChooser<>();
    private static SendableChooser<AbstractControl> driveMode = new SendableChooser<AbstractControl>();
    
    // Creates default control var
    private static AbstractControl _defaultControl;

    /**
     * Initializes the control object
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public static void initialize (AbstractControl defaultControl, AbstractControl... controls) {
        _defaultControl = defaultControl;


        // Sets up driver orientation options //TODO: do you need to add the default option (and what it means for the abstractcontrols setup)
        driverOrientation.addOption("Competition", DriverOrientation.XAxisTowardsTrigger);
        driverOrientation.addOption("Y Axis", DriverOrientation.YAxisTowardsTrigger);
        driverOrientation.setDefaultOption("Competition", DriverOrientation.XAxisTowardsTrigger);

        // Sets up drive mode options
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }
        driveMode.setDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

        // Sets up auto routine options
        EnumSet.allOf(AutoName.class)
            .forEach(v -> autoChooser.addOption(v.pathref, v));
        autoChooser.setDefaultOption(AutoName.Speaker_2_note.pathref, AutoName.Speaker_2_note);


        // Creates the UI for driverOrientation
        ShuffleboardUI.Overview.getTab()
            .add("Driver Orientation", driverOrientation)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        // Creates the UI for drive mode
        ShuffleboardUI.Overview.getTab()
            .add("Drive Mode", driveMode)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 1)
            .withSize(3, 1);

        // Creates the UI for auto routines
        ShuffleboardUI.Autonomous.getTab()
            .add("Auto Code", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(3, 1)
            .withPosition(6, 1);
    }


    public static DriverOrientation getDriverOrientation() {
        return driverOrientation.getSelected();
    }

    public static String getAutoChooser() {
        return autoChooser.getSelected().pathref;
    }

    public static AbstractControl getDriveControl() {
        if(driveMode.getSelected() == null)
            return _defaultControl;
        return driveMode.getSelected();
    }
}