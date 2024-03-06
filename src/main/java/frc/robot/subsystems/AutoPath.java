package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.OrientTowards;
import frc.robot.commands.auto.PushAmp;
import frc.robot.commands.auto.PushSpeaker;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.commands.notes.ShootAmp;
import frc.robot.commands.notes.ShootSpeaker;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.OrientTarget;

public class AutoPath {

    private final SendableChooser<Command> autoChooser;

    public AutoPath(Drivetrain drivetrain, Acquisition acquisition, Photosensor photosensor, Channel channel,
            Shooter shooter, Crashbar crashbar) {
        NamedCommands.registerCommand("Stop", new InstantCommand(() -> {
            drivetrain.stop();
        }));

        NamedCommands.registerCommand("PushSpeaker", new PushSpeaker(channel, shooter));
        NamedCommands.registerCommand("FlywheelSpeaker", new InstantCommand(()->shooter.toggle(ShooterStates.SPEAKER)));
        NamedCommands.registerCommand("PushAmp", new PushAmp(channel, shooter, crashbar));
        NamedCommands.registerCommand("FlywheelAmp", new InstantCommand(()->{
            shooter.toggle(ShooterStates.AMP);
            crashbar.toggle(CrashbarStates.EXTENDED);
        }));
        NamedCommands.registerCommand("OrientSpeaker", new OrientTowards(drivetrain, OrientTarget.Speaker));
        NamedCommands.registerCommand("OrientAmp", new OrientTowards(drivetrain, OrientTarget.Amp));
        NamedCommands.registerCommand("Acquire", new AcquireSmart(acquisition, channel, photosensor));

        AutoBuilder.configureHolonomic(
                drivetrain.poseFilter::getEstimatedPosition, // Robot pose supplier
                drivetrain::setToPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> {
                    SmartDashboard.putNumberArray("speeds",
                            new double[] { speeds.vxMetersPerSecond, speeds.vyMetersPerSecond });
                    drivetrain
                            .drive(new DriveCommandData(speeds.vxMetersPerSecond,
                                    speeds.vyMetersPerSecond,
                                    speeds.omegaRadiansPerSecond, false));
                }, // Method that will drive the robot given ROBOT
                   // RELATIVE
                Constants.Swerve.PathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain // Reference to this subsystem to set requirements
        );

        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void putAutoChooser() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutoChooserSelected() {
        return autoChooser.getSelected();
    }
}
