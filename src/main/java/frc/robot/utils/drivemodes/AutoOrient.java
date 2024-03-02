package frc.robot.utils.drivemodes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.utils.DriverStationUtils;

public class AutoOrient {

    // Init variables
    private static PIDController anglePID = new PIDController(2, 0, 0);

    public AutoOrient() {
        // Enables continious input for PID
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

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

    /**
     * runs calculations for auto-orient
     * 
     * @return
     *         DriveCommandData object with drive directions
     */

    public double calculateSpeaker(Pose2d swerve_position) {
        Translation2d targetPos = new Translation2d(0, 0);
        targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
            ? Constants.FieldConstants.TEAM_RED_SPEAKER
            : Constants.FieldConstants.TEAM_BLUE_SPEAKER;
        return calculate(targetPos, swerve_position);
    }

    public double calculateAmp(Pose2d swerve_position) {
        Rotation2d targetRotation = new Rotation2d(0);
        targetRotation = DriverStationUtils.getCurrentAlliance() == Alliance.Red
            ? Constants.FieldConstants.TEAM_RED_AMP
            : Constants.FieldConstants.TEAM_BLUE_AMP;
        return calculate(targetRotation, swerve_position);
    }


    public double calculate(Translation2d targetPos, Pose2d swerve_position) {
        double spin;
        if (shouldUpdateAngle(swerve_position.getTranslation(), targetPos)) {
            spin = anglePID.calculate(swerve_position.getRotation().getRadians(),
                    AutoOrient.rotationFacingTarget(swerve_position.getTranslation(), targetPos).getRadians());
        } else {
            spin = 0;
        }
        // Returns spin
        return spin;
    }


    public double calculate(Rotation2d targetAngle, Pose2d swerve_position) {
        double spin;
        double swerve_angle_radians = swerve_position.getRotation().getRadians();
        double target_angle_radians = targetAngle.getRadians();
        if (Math.abs(swerve_angle_radians - target_angle_radians) > 0.04) {
            spin = anglePID.calculate(swerve_angle_radians, target_angle_radians);
        } else {
            spin = 0;
        }
        // Returns spin
        return spin;
    }
}
