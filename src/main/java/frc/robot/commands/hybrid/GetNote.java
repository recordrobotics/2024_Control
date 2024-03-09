package frc.robot.commands.hybrid;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Vision;
import frc.robot.utils.DriveCommandData;


public class GetNote extends Command{
    Drivetrain driveTrain;
    Vision vision;
    double speed;
    Photosensor photosensor;

    public GetNote(Drivetrain drivetrain, Vision vision, double speed, Photosensor photosensor){
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        this.driveTrain = drivetrain;
        this.vision = vision;
        this.speed = speed;
        this.photosensor = photosensor;
    }

    @Override
    public void execute(){
        driveTrain.drive(new DriveCommandData(speed, 0, 0, false));
    }

    @Override
    public boolean isFinished(){
        return photosensor.getDebouncedValue();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
