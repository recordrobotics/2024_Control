package frc.robot.shuffleboard;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoName;
import frc.robot.Constants.FieldStartingLocation;

public class AutonomousLayout extends Layout {

    private static final Field2d field = new Field2d();
    private static SendableChooser<AutoName> autoChooser = new SendableChooser<>();
    private static SendableChooser<FieldStartingLocation> fieldStartingLocationChooser = new SendableChooser<>();

    private Supplier<Boolean> acquisitionValue = () -> null;
    private Supplier<Boolean> hasNoteValue =  () -> false;

    public AutonomousLayout() {
        getTab()
            .add(field)
            .withWidget(BuiltInWidgets.kField)
            .withSize(6, 4)
            .withPosition(0, 0);

        // Creates the UI for auto routines
        getTab()
            .add("Auto Code", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(3, 1)
            .withPosition(6, 1);

        // Creates the UI for starting location
        getTab()
            .add("Starting Location", fieldStartingLocationChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(3, 1)
                .withPosition(6, 0);
            
        getTab()
            .addBoolean("Acquisition",  ()->acquisitionValue.get())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(9, 0)
                .withSize(1, 1);
            
        getTab()
            .addBoolean("Has Note", ()->hasNoteValue.get())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(9, 1)
            .withSize(1, 1);
    }

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setAcquisition(Supplier<Boolean> acquisition) {
        acquisitionValue = acquisition;
    }

    public void setHasNote(Supplier<Boolean> hasNote) {
        hasNoteValue = hasNote;
    }
    
    @Override
    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Autonomous");
    }

    public String getAutoChooser() {
        return autoChooser.getSelected().pathref;
    }

    public FieldStartingLocation getStartingLocation() {
        if(fieldStartingLocationChooser.getSelected() == null)
            return FieldStartingLocation.FrontSpeaker;
        return fieldStartingLocationChooser.getSelected();
    }
}