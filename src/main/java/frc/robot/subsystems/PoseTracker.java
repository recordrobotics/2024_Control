package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PoseTracker extends SubsystemBase {

    // Creates Nav object
    private final NavSensor nav = new NavSensor();

    // Creates swerve post estimation filter
    public static SwerveDrivePoseEstimator poseFilter;

    public PoseTracker(
            SwerveDriveKinematics kinematics,
            Pose2d startingPose) {
        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                kinematics,
                nav.getAdjustedAngle(),
                getModulePositions(),
                ShuffleboardUI.Autonomous.getStartingLocation().getPose());
    }

    @Override
    public void periodic() {
        poseFilter.update(nav.getAdjustedAngle(), getModulePositions());
        poseFilter.addVisionMeasurement(limelight.something???, how do i get a timestamp???);

        SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
        SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
        ShuffleboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
    }

    private SwerveModulePosition[] getModulePositions() {
        return drivetrain.getModulePositions();
    }

    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    public void updateOdometry(SwerveModulePosition[] positions) {
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

    /** Similar to resetPose but adds an argument for the initial pose */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(
                nav.getAdjustedAngle(),
                getModulePositions(),
                pose);
    }

    /** Resets the field relative position of the robot (mostly for testing). */
    public void resetStartingPose() {
        poseFilter.resetPosition(
                nav.getAdjustedAngle(),
                getModulePositions(),
                ShuffleboardUI.Autonomous.getStartingLocation().getPose());
    }

    /** Resets the pose to FrontSpeakerClose (shooter facing towards speaker) */
    public void resetDriverPose() {
        poseFilter.resetPosition(
                nav.getAdjustedAngle(),
                getModulePositions(),
                Constants.FieldStartingLocation.FrontSpeakerClose.getPose());
    }
}
