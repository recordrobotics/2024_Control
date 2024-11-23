package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.SimpleMath;

public class Limelight extends SubsystemBase {

  private static final String name = "limelight";
  public int numTags = 0;
  public double confidence = 0;
  public boolean hasVision = false;
  public boolean limelightConnected = false;

  private Drivetrain drivetrain;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    LimelightHelpers.setPipelineIndex(name, 0);
  }

  @Override
  public void periodic() {
    confidence = 0;
    LimelightHelpers.SetRobotOrientation(
        name,
        Drivetrain.poseFilter.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    LimelightHelpers.PoseEstimate measurement_m2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (measurement == null || measurement_m2 == null) {
      limelightConnected = false;
      return;
    } else {
      limelightConnected = true;
    }

    numTags = measurement.tagCount;

    if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
      if (measurement.avgTagDist
          < Units.feetToMeters(7)) { // 7 feet is where the MT1 (yellow) gets bad wiggles
        confidence = 0.65; // mt 1
      } else {
        confidence = 0.7; // mt 2
        measurement = measurement_m2;
      }
    }

    // measurement.pose = new Pose2d(
    //     measurement.pose.getTranslation(),
    //     measurement.pose.getRotation().plus(Rotation2d.fromDegrees(180))
    //     );

    double timeSinceAuto = Timer.getFPGATimestamp() - Robot.getAutoStartTime();

    if (timeSinceAuto > 1
        && measurement
                .pose
                .getTranslation()
                .getDistance(Drivetrain.poseFilter.getEstimatedPosition().getTranslation())
            > 2) {
      confidence = 0;
    }
  }

  /** frees up all hardware allocations */
  public void close() {}
}
