package frc.robot.utils.drivemodes.spin;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;

public class XboxDPad {

    private PIDController anglePID = new PIDController(3.36, 0, 0);

    public XboxDPad() {
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean shouldExecute(DoubleControl _controls) {
        boolean is_pressed = _controls.getAngleDPad().getSecond();
        return is_pressed;
    }

    public double calculate(DoubleControl _controls, Pose2d swerve_position) {
        double spin = anglePID.calculate(swerve_position.getRotation().getRadians(),
                _controls.getAngleDPad().getFirst().getRadians());

        return spin;
    }

}