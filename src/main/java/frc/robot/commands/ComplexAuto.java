package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class ComplexAuto extends SequentialCommandGroup {

    private Drivetrain _drivetrain;
    
    public ComplexAuto (Drivetrain _Drivetrain) {
        addCommands(
            new MoveDistance(_drivetrain, new Translation2d(1, 0)),
            new MoveDistance(_drivetrain, new Translation2d(0, 1))
        );
    }

}