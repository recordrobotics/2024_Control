package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Vision;
import frc.robot.utils.DriveCommandData;

public class NoteOrient extends Command{

    Drivetrain driveTrain;
    Vision vision;
    PIDController anglePID;
    DoubleControl controls;
    Photosensor photosensor;

    public NoteOrient(Drivetrain drivetrain, Vision vision, DoubleControl controls, Photosensor photosensor){
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        this.driveTrain = drivetrain;
        this.vision = vision;
        this.controls = controls;
        this.photosensor = photosensor;

        anglePID = new PIDController(0.4, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        driveTrain.stop();
    }

    @Override
    public void execute() {
        driveTrain.drive(new DriveCommandData(0, 0, anglePID.calculate(vision.ringDirection().getRadians()), false));
    }

    @Override
    public boolean isFinished(){
        return vision.ringDirection().getRadians() < 0.1 && vision.ringDirection().getRadians() > -0.1;
    }

    @Override
    public void end(boolean interrupted) {
        new GetNote(driveTrain, vision, controls, 0.1, photosensor);
    }
}