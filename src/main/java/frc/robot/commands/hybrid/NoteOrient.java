package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utils.DriveCommandData;

public class NoteOrient extends Command{

    Drivetrain driveTrain;
    Vision vision;
    PIDController anglePID;
    DoubleControl controls;

    public NoteOrient(Drivetrain drivetrain, Vision vision, DoubleControl controls){
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        this.driveTrain = drivetrain;
        this.vision = vision;
        this.controls = controls;

        anglePID = new PIDController(0.4, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void initialize() {
        driveTrain.stop();
    }

    public void execute() {
        driveTrain.drive(new DriveCommandData(0, 0, anglePID.calculate(vision.ringDirection().getRadians()), false));
    }

    public boolean isFinished(){
        return controls.getKillAuto();
    }
}
