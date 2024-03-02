package frc.robot.utils.drivemodes.spin;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;

public class XboxDPad {

    private AutoOrient _autoOrient;

    public XboxDPad() {
        _autoOrient = new AutoOrient();
    }

    public boolean shouldExecute(DoubleControl _controls) {
        boolean is_pressed = _controls.getAngleDPad().getSecond();
        return is_pressed;
    }

    public double calculate(DoubleControl _controls, Pose2d swerve_position) {
        return _autoOrient.calculate(_controls.getAngleDPad().getFirst(), swerve_position);
    }

}