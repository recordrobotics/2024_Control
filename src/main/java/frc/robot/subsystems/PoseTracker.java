package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PoseTracker extends SubsystemBase {
  public static PoseTracker instance;

  public final NavSensor nav = new NavSensor(); // TODO should be seperate subsystem or private

  private static SwerveDrivePoseEstimator poseFilter;

  public PoseTracker() {
    nav.resetAngleAdjustment();

    poseFilter =
        new SwerveDrivePoseEstimator(
            Drivetrain.instance.getKinematics(),
            nav.getAdjustedAngle(),
            getModulePositions(),
            ShuffleboardUI.Autonomous.getStartingLocation().getPose());
  }

  @Override
  public void periodic() {
    poseFilter.update(nav.getAdjustedAngle(), getModulePositions());
    poseFilter.addVisionMeasurement(
        Limelight.instance.getPoseEstimate().pose,
        Limelight.instance.getPoseEstimate().timestampSeconds,
        VecBuilder.fill(
            Limelight.instance.getConfidence(),
            Limelight.instance.getConfidence(),
            9999999) // big number to remove all influence of limelight pose rotation
        );

    SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
    SmartDashboard.putNumber("pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
    ShuffleboardUI.Autonomous.setRobotPose(poseFilter.getEstimatedPosition());
  }

  private SwerveModulePosition[] getModulePositions() {
    return Drivetrain.instance.getModulePositions();
  }

  public Pose2d getEstimatedPosition() {
    return poseFilter.getEstimatedPosition();
  }

  /** Similar to resetPose but adds an argument for the initial pose */
  public void setToPose(Pose2d pose) {
    poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
  }

  /** Resets the field relative position of the robot (mostly for testing). */
  public void resetStartingPose() {
    setToPose(ShuffleboardUI.Autonomous.getStartingLocation().getPose());
  }

  /** Resets the pose to FrontSpeakerClose (shooter facing towards speaker) */
  public void resetDriverPose() {
    poseFilter.resetPosition(
        nav.getAdjustedAngle(),
        getModulePositions(),
        Constants.FieldStartingLocation.FrontSpeakerClose.getPose());
  }
}
