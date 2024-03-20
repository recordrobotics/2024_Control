package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TestLayout extends Layout {
    @Override
    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Test");
    }
}