package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utils.DriveCommandData;

public class AutoPath {

    private final SendableChooser<Command> autoChooser;

    public AutoPath(Drivetrain drivetrain) {
        AutoBuilder.configureHolonomic(
                drivetrain.poseFilter::getEstimatedPosition, // Robot pose supplier
                drivetrain::setToPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> {
                    SmartDashboard.putNumberArray("speeds",
                            new double[] { speeds.vxMetersPerSecond, speeds.vyMetersPerSecond });
                    drivetrain
                            .drive(new DriveCommandData(MathUtil.clamp(speeds.vxMetersPerSecond, -0.05, 0.05),
                                    MathUtil.clamp(speeds.vyMetersPerSecond, -0.05, 0.05),
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
