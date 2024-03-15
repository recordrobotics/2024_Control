package frc.robot.utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.PushAmp;
import frc.robot.commands.auto.PushSpeaker;
import frc.robot.commands.auto.SetupAmp;
import frc.robot.commands.auto.SetupSpeaker;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.subsystems.*;

public class AutoPath {

    public AutoPath(Drivetrain drivetrain, 
                    Acquisition acquisition, 
                    Photosensor photosensor, 
                    Channel channel,
                    Shooter shooter, 
                    Crashbar crashbar) {

        // Registering named commands (so that the pathplanner can call them by name)
        NamedCommands.registerCommand("Stop", new InstantCommand(() -> drivetrain.stop()));
        NamedCommands.registerCommand("Acquire", new AcquireSmart(acquisition, channel, photosensor, shooter));
        NamedCommands.registerCommand("PushSpeaker", new PushSpeaker(channel, shooter));
        NamedCommands.registerCommand("FlywheelSpeaker", new SetupSpeaker(shooter));
        NamedCommands.registerCommand("PushAmp", new PushAmp(channel, shooter, crashbar));
        NamedCommands.registerCommand("FlywheelAmp", new SetupAmp(shooter, crashbar, true));

        // Configures auto builder
        AutoBuilder.configureHolonomic(
                drivetrain.poseFilter::getEstimatedPosition, // Robot pose supplier
                drivetrain::setToPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                
                // Method that will drive the robot given ROBOT RELATIVE speeds
                (speeds) -> {
                    SmartDashboard.putNumberArray("speeds",
                            new double[] { speeds.vxMetersPerSecond, speeds.vyMetersPerSecond });
                    drivetrain
                            .drive(new DriveCommandData(speeds.vxMetersPerSecond,
                                    speeds.vyMetersPerSecond,
                                    speeds.omegaRadiansPerSecond, false));
                },
                
                // com.pathplanner.lib.util.HolonomicPathFollowerConfig for configuring the path following commands
                Constants.Swerve.PathFollowerConfig,

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                
                // Reference to this subsystem to set requirements
                drivetrain
        );
    }
}