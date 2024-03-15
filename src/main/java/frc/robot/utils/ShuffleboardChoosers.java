package frc.robot.utils;
import java.util.EnumSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.control.AbstractControl;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShuffleboardChoosers {

    // Sets up sendable choosers
    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();
    public static SendableChooser<AutoName> autoChooser = new SendableChooser<>();
    private static SendableChooser<AbstractControl> driveMode = new SendableChooser<AbstractControl>();
    public static SendableChooser<FieldStartingLocation> fieldStartingLocationChooser = new SendableChooser<>();
    
    // Creates default control var
    private static AbstractControl _defaultControl;

    /**
     * Initializes the control object
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public static void initialize (AbstractControl defaultControl, AbstractControl... controls) {
        _defaultControl = defaultControl;

        // Sets up drive mode options
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }
        driveMode.setDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

        // Sets up auto routine options
        EnumSet.allOf(AutoName.class)
            .forEach(v -> autoChooser.addOption(v.pathref, v));
        autoChooser.setDefaultOption(AutoName.Speaker_2_note.pathref, AutoName.Speaker_2_note);

        // Sets up auto routine options
        EnumSet.allOf(DriverOrientation.class)
            .forEach(v -> driverOrientation.addOption(v.display_name, v));
        driverOrientation.setDefaultOption(DriverOrientation.XAxisTowardsTrigger.display_name, DriverOrientation.XAxisTowardsTrigger);

        // Sets up starting pose options
        EnumSet.allOf(FieldStartingLocation.class)
            .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.setDefaultOption(FieldStartingLocation.AtAmp.toString(), FieldStartingLocation.AtAmp);


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

        ShuffleboardUI.Autonomous.getTab()
            .add("Starting Location", fieldStartingLocationChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(3, 1)
            .withPosition(6, 0);
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

    public static FieldStartingLocation getStartingLocation() {
        return fieldStartingLocationChooser.getSelected();
    }



    // Orient stuff
    public enum DriverOrientation {
        XAxisTowardsTrigger("Competition"),
        YAxisTowardsTrigger("Y Axis")
        ;

        public final String display_name;

        DriverOrientation(String orientation) {
            display_name = orientation;
        }
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

    public enum FieldStartingLocation {
        FrontSpeaker(
                // Red
                new Pose2d(14.169, 5.584, Rotation2d.fromDegrees(180)),
                // Blue
                new Pose2d(2.371, 5.584, Rotation2d.fromDegrees(0))),
        FrontSpeakerClose(
                // Red
                new Pose2d(14.169, 5.584, Rotation2d.fromDegrees(180)),
                // Blue
                new Pose2d(2.371, 5.584, Rotation2d.fromDegrees(0))),
        AtAmp(
                // Red
                new Pose2d(14.89, 7.27, Rotation2d.fromDegrees(-90)),
                // Blue
                new Pose2d(1.65, 7.27, Rotation2d.fromDegrees(-90))),
        DiagonalSpeaker(
                // Red
                new Pose2d(16.268, 4.454, Rotation2d.fromDegrees(-120.665)),
                // Blue
                new Pose2d(1.564, 4.403, Rotation2d.fromDegrees(-60.849))),
        DiaAmpSpeaker(
                // Red
                new Pose2d(16.199, 6.730, Rotation2d.fromDegrees(119.8)),
                // Blue
                new Pose2d(1.776, 6.703, Rotation2d.fromDegrees(58.762))),
                ;

        private final Pose2d m_transformRed;
        private final Pose2d m_transformBlue;

        private FieldStartingLocation(Pose2d poseRed, Pose2d poseBlue) {
            m_transformRed = poseRed;
            m_transformBlue = poseBlue;
        }

        public Pose2d getPose() {
            return DriverStationUtils.getCurrentAlliance() == Alliance.Red ? m_transformRed : m_transformBlue;
        }
    }
}