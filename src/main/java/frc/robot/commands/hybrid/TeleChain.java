package frc.robot.commands.hybrid;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climbers.ClimberStates;
import frc.robot.Constants.FieldPosition;
import frc.robot.utils.TriggerProcessor.TriggerDistance;


@TriggerDistance(distance = 5, position = FieldPosition.CenterChain)
public class TeleChain extends SequentialCommandGroup {

    // Load the path we want to pathfind to and follow
    private PathPlannerPath[] paths;

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    private PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    private static Climbers _climbers;

    public TeleChain (Climbers climbers) {
        _climbers = climbers;

        // Loads path
        paths = new PathPlannerPath[] {
            PathPlannerPath.fromPathFile("SpeakerSideChain"),
            PathPlannerPath.fromPathFile("AmpSideChain"),
            PathPlannerPath.fromPathFile("FarSideChain"),
        };

        // Gets path closest to the robot
        // TODO: there is most definitely a better way to do this. Find it (possible use translation2d.getnearest())
        PathPlannerPath lowest_path = paths[0];
        double lowest_distance = 1000;
        for (PathPlannerPath path: paths) {
            Translation2d swerve_translation = Drivetrain.poseFilter.getEstimatedPosition().getTranslation();
            Translation2d path_translation = path.getPreviewStartingHolonomicPose().getTranslation();
            double distance = swerve_translation.getDistance(path_translation);
            if (distance < lowest_distance)
                lowest_distance = distance;
                lowest_path = path;
        }

        addCommands(
            new InstantCommand(()->_climbers.toggle(ClimberStates.UP), _climbers),
            Drivetrain.poseFilter.waitUntilCertain(),
            new WaitCommand(1),
            AutoBuilder.pathfindThenFollowPath(lowest_path, constraints, 0),
            new InstantCommand(()->_climbers.toggle(ClimberStates.DOWN), _climbers)
        );
    }
}  