package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PoseTracker extends SubsystemBase {
    public static PoseTracker instance;

    private final NavSensor nav = new NavSensor();

    public static SwerveDrivePoseEstimator poseFilter;

    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public PoseTracker(
            Drivetrain drivetrain,
            Limelight limelight) {
        this.drivetrain = drivetrain;        
        this.limelight = limelight;

        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                drivetrain.getKinematics(),
                nav.getAdjustedAngle(),
                getModulePositions(),
                ShuffleboardUI.Autonomous.getStartingLocation().getPose());
    }

    @Override
    public void periodic() {
        poseFilter.update(nav.getAdjustedAngle(), getModulePositions());
        poseFilter.addVisionMeasurement(
            limelight.getPoseEstimate().pose,
            limelight.getPoseEstimate().timestampSeconds,
            VecBuilder.fill(
                limelight.getConfidence(),
                limelight.getConfidence(),
                9999999) // big number to remove all influence of limelight pose rotation
        );

        limelight.poseTrackerEstimatedPose = poseFilter.getEstimatedPosition();

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

    /**
     * Add a vision measurement to the pose filter. The confidence value is the standard deviation
     * of the measurement, so larger values mean less confidence.
     *
     * @param estimate    The pose estimate from the vision system.
     * @param confidence  The confidence in the measurement, in the range [0, 1].
     */
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
