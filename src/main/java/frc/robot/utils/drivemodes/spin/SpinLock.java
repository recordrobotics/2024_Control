package frc.robot.utils.drivemodes.spin;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SpinLock {
    private AutoOrient _autoOrient;
    private Rotation2d lockedAngle;

    public SpinLock() {
        _autoOrient = new AutoOrient();
    }

    public void setAngle(Rotation2d rotation) {
        lockedAngle = rotation;
    }

    public double calculate(Pose2d swerve_position) {
        return _autoOrient.calculate(lockedAngle, swerve_position);
    }

}