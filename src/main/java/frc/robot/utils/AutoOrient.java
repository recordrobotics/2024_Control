package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoOrient {

    /**
     * Calculates the correct angle to move the robot to in order to face the target
     * position
     * 
     * @param robotPosition
     * @param targetPosition
     * @return
     */
    public static Rotation2d rotationFacingTarget(Translation2d robotPosition, Translation2d targetPosition) {
        return new Rotation2d(
                Math.atan2(robotPosition.getY() - targetPosition.getY(), robotPosition.getX() - targetPosition.getX()));
    }

    public static boolean shouldUpdateAngle(Translation2d robotPosition, Translation2d targetPosition) {
        return robotPosition.getDistance(targetPosition) > 0.05; // TODO: just a placeholder value, change later
    }
}
