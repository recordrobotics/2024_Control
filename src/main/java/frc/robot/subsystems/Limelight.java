package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.SimpleMath;

public class Limelight extends SubsystemBase {
    
    private static final String name = "limelight";
    private final NavSensor _nav = new NavSensor();
    private int numTags = 0;
    private double confidence = 0;
    private boolean hasVision = false;
    private boolean limelightConnected = false;

    private Drivetrain drivetrain;

    public Limelight(Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        LimelightHelpers.setPipelineIndex(name, 0);
        ShuffleboardUI.Overview.setTagNum(()->numTags);
        ShuffleboardUI.Overview.setConfidence(()->confidence);
        ShuffleboardUI.Overview.setHasVision(()->hasVision);
        ShuffleboardUI.Overview.setLimelightConnected(()->limelightConnected);
    }

    @Override
    public void periodic() {
        confidence = 0;
        LimelightHelpers.SetRobotOrientation(name, _nav.getAdjustedAngle().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        LimelightHelpers.PoseEstimate measurement_m2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        if(measurement == null || measurement_m2 == null){
            limelightConnected = false;
            return;
        } else {
            limelightConnected = true;
        }
            
        numTags = measurement.tagCount;

        // TODO: add check if measurement.pose is close to current position estimate
        if(measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)){
            confidence = 0.7; // 0.5 for mt 1
            measurement = measurement_m2;
        }

        //measurement.pose = measurement.pose.rotateBy(Rotation2d.fromDegrees(180));
        handleMeasurement(measurement, confidence);
    }

    private void handleMeasurement(LimelightHelpers.PoseEstimate estimate, double confidence){
        if(confidence > 0){
            hasVision = true;
            ShuffleboardUI.Autonomous.setVisionPose(estimate.pose);
            drivetrain.addVisionMeasurement(estimate, confidence);
        } else {
            hasVision = false;
            ShuffleboardUI.Autonomous.setVisionPose(new Pose2d());
        }
    }

}
