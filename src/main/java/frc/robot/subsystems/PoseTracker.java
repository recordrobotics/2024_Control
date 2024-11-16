package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PoseTracker extends SubsystemBase {

    // Creates Nav object
    private final NavSensor nav = new NavSensor();

    // Creates swerve post estimation filter
    public static SwerveDrivePoseEstimator poseFilter;

    public PoseTracker(
            SwerveDriveKinematics kinematics,
            SwerveModulePosition[] startingPositions,
            Pose2d startingPose) {
        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                kinematics,
                nav.getAdjustedAngle(),
                startingPositions,
                startingPose);
    }

    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
        SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
        ShuffleboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
    }

    public void update(SwerveModulePosition[] positions) {
        poseFilter.update(
                nav.getAdjustedAngle(),
                positions);
    }

    public void addVisionMeasurement(LimelightHelpers.PoseEstimate estimate, double confidence) {
        poseFilter.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(
                        confidence,
                        confidence,
                        9999999) // big number to remove all influence of limelight pose rotation
        );
    }
}
