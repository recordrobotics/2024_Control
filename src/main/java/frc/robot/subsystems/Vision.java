package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
    
    private static PhotonCamera camera;
    private static PhotonPipelineResult latestResult;
    private static Pose2d latestEstimatedPose;
	
    public Vision() {
        camera = new PhotonCamera(Constants.Vision.cameraID);
    }

    public void periodic()  {
        // Gets a frame from the camera
		latestResult = camera.getLatestResult();

        // Stores latest estimation every time
        if (latestResult.hasTargets()) {
            latestEstimatedPose = estimateGlobalPose();
        }
    }

    public Pose2d getLatestEstimatedPose() {
        return latestEstimatedPose;
    }

	public static Pose2d estimateGlobalPose() {

		// Gets target object from apriltag perspective photonvision
		PhotonTrackedTarget target = latestResult.getBestTarget();

		// Gets the Transform3d object for april to robot
		Transform3d robot_to_april = target.getBestCameraToTarget()/*.plus(robotToCam.inverse())*/; // you could put the offset here if you were testing for reals
		Transform3d april_to_robot = robot_to_april.inverse();

		// Gets the fiducial ID and uses it to get the correct transform 2d object
		int targetID = target.getFiducialId();
		Transform3d global_to_april = Constants.Vision.tagTransforms[targetID - 1];

		// Adds the two transform objects together to get robot to global
		Transform3d global_to_robot = global_to_april.plus(april_to_robot);

        // Calculates the robot's global pose
        Pose2d robotGlobalPose = new Pose2d(
            new Translation2d(global_to_robot.getX(), global_to_robot.getY()),
            new Rotation2d(global_to_robot.getRotation().getZ())
        );

        // Returns
        return robotGlobalPose;
    }

    public boolean checkForSpecificTags(Integer[] allowedTags) {
        if (latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            int targetID = target.getFiducialId() - 1;
            return Arrays.asList(allowedTags).contains(targetID);
        }
        return false;
    }

	// public static boolean checkForTarget() {
	// 	var result = camera.getLatestResult();//get a frame from the camera
	// 	boolean hasTargets = result.hasTargets();//check for targets. This MUST be checked for, otherwise an error will occur if there isn't a target.
	// 	return hasTargets;
	// }

    // public static double getTargetID() {
    //     var result = camera.getLatestResult();
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     int targetID = target.getFiducialId();
    //     return targetID;
    // }

}