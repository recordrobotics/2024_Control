package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;

public class AutoDrive extends Command {

    private Drivetrain _drivetrain;

    private Translation2d target;

    public AutoDrive(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        _drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        target = pose.getTranslation().plus(new Translation2d(1, 1));
    }

    @Override
    public void execute() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        Translation2d swerve_position = pose.getTranslation();

        double x_diff = target.getX() - swerve_position.getX();
        double y_diff = target.getY() - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
        double CLAMP_DISTANCE = 0.5;
        double clamped_magnitude = Math.min(1, magnitude / CLAMP_DISTANCE);

        // Calculates x and y speeds from magnitude and diff
        double speed = 0.1;
        double x_speed = x_diff / magnitude * speed * clamped_magnitude;
        double y_speed = y_diff / magnitude * speed * clamped_magnitude;

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                x_speed, // x_speed,
                y_speed, // y_speed,
                0,
                true);

        _drivetrain.drive(driveCommandData);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        Translation2d swerve_position = pose.getTranslation();

        double x_diff = target.getX() - swerve_position.getX();
        double y_diff = target.getY() - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);

        return magnitude < 0.05;
    }

}
