package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.utils.ApriltagMeasurement;


public class Vision extends SubsystemBase {

    private static NetworkTable _networkTable;

    public Vision() {
        _networkTable = NetworkTableInstance.getDefault().getTable("JetsonVision");
    }

    public Optional<ApriltagMeasurement> getMeasurement() {

        // If no measurement or measurement does not exist, return Optional.empty()
        try {
            if (!_networkTable.getValue("Has pose").getBoolean()) 
                return Optional.empty();
        } catch (Exception e) {
            return Optional.empty();
        }

        // Gets last pose
        NetworkTableValue lastMeasurement = _networkTable.getValue("Pose");
        double[] poseArray = lastMeasurement.getDoubleArray();

        // Gets last time
        long lastDetectedTime = lastMeasurement.getTime();
        double latency = poseArray[4];
        long timeStamp = lastDetectedTime - (long) latency;
        
        // Gets pose data
        double tagX = poseArray[0];
        double tagY = poseArray[1];
        double tagRot = poseArray[2];
        int tagID = (int) poseArray[3];
        // Gets last detected pose
        Pose2d pose = new Pose2d(
                new Translation2d(tagX, tagY),
                new Rotation2d(tagRot));

        // Creates an Apriltag measurement based off of the data and returns
        ApriltagMeasurement lastApriltagMeasurement = new ApriltagMeasurement(pose, timeStamp, tagRot, tagID);
        return Optional.of(lastApriltagMeasurement);

    }

}