package frc.robot.utils.drivemodes;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;


public class SpinDrive {

    // Init variables
    private static PIDController anglePID = new PIDController(0.4, 0, 0);

    public SpinDrive() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * runs calculations for auto-orient
     * @return 
     * DriveCommandData object with drive directions
     */
    public static double calculate(DoubleControl _controls, Pose2d swerve_position) {

        // Calculates spin (I don't know how  it works, TODO: get vlad to explain to me)
        double target_rotation = _controls.getSpinKnob();
        double swerve_rotation = swerve_position.getRotation().getRadians();
        double PID_calculation = anglePID.calculate(swerve_rotation, target_rotation);
        double spin = MathUtil.clamp(PID_calculation, -0.5, 0.5);

        // Returns spin
        return spin;
    }
}