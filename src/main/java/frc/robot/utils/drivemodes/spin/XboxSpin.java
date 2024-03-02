package frc.robot.utils.drivemodes;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.DoubleControl;
import frc.robot.utils.SimpleMath;

public class XboxSpin {

    // Init variables
    private PIDController anglePID = new PIDController(3.36, 0, 0);

    public XboxSpin() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static boolean shouldUpdateAngle(Translation2d robotPosition, Translation2d targetPosition) {
        return robotPosition.getDistance(targetPosition) > 0.05; // TODO: just a placeholder value, change later
    }

    public boolean shouldExecute(DoubleControl _controls) {
        boolean is_over_threshold = _controls.getXboxSpinAngle().getSecond() > 0;
        return is_over_threshold; 
    }

    public double calculate(DoubleControl _controls, Pose2d swerve_position) {

        double xbox_magnitutde = _controls.getXboxSpinAngle().getSecond();
        double scaled_magnitude = SimpleMath.Remap(xbox_magnitutde, 0, 1);

        double xbox_angle = _controls.getXboxSpinAngle().getFirst();
        double robot_angle = swerve_position.getRotation().getRadians();

        double spin = anglePID.calculate(robot_angle, xbox_angle);
            
        // Returns spin
        return spin * scaled_magnitude;
    }

}