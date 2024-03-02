package frc.robot.utils.drivemodes.spin;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;

public class KnobSpin {

    // Init variables
    private PIDController anglePID = new PIDController(3.36, 0, 0);

    public KnobSpin() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * runs calculations for auto-orient
     * 
     * @return
     *         DriveCommandData object with drive directions
     */
    public double calculate(DoubleControl _controls, Pose2d swerve_position) {

        // Calculates spin (I don't know how it works, TODO: get vlad to explain to me)
        double target_rotation = _controls.getSpinKnob();
        double swerve_rotation = swerve_position.getRotation().getRadians();
        double PID_calculation = anglePID.calculate(swerve_rotation, target_rotation);
        double spin = PID_calculation;

        // Returns spin
        return spin;
    }
}