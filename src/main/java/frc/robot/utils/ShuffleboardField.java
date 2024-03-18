package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ShuffleboardField {
    private static final Field2d field1 = new Field2d();

    static {
        ShuffleboardTab tab = ShuffleboardUI.Autonomous.getTab();
        var fieldWidget = tab.add(field1);
        fieldWidget.withWidget(BuiltInWidgets.kField);
        fieldWidget.withSize(6, 4);
        fieldWidget.withPosition(0, 0);
    }

    public static void setRobotPose(Pose2d pose) {
        field1.setRobotPose(pose);
    }
}