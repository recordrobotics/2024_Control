package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Drivetrain;

public class PlannedAuto extends SequentialCommandGroup {
    public PlannedAuto(Drivetrain drivetrain, AutoPath autoPath) {
        addCommands(
                new PathPlannerAuto("Amp and Speaker"),
                new StopAndWait(drivetrain, 1));
    }

}