package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.OrientTarget;
import frc.robot.utils.drivemodes.spin.AutoOrient;

public class OrientTowards extends Command {
    private final Drivetrain _drivetrain;

    private AutoOrient _autoOrient;

    private Translation2d targetPos;
    private Rotation2d targetRot;

    public OrientTowards(Drivetrain drivetrain, Translation2d target) {
        _autoOrient = new AutoOrient();
        this.targetPos = target;
        this._drivetrain = drivetrain;
    }

    public OrientTowards(Drivetrain drivetrain, Rotation2d target) {
        _autoOrient = new AutoOrient();
        this.targetRot = target;
        this._drivetrain = drivetrain;
    }

    public OrientTowards(Drivetrain drivetrain, OrientTarget target) {
        _autoOrient = new AutoOrient();
        this._drivetrain = drivetrain;
        switch (target) {
            case Speaker:
                this.targetPos = DriverStationUtils.getCurrentAlliance() == Alliance.Red
                        ? Constants.FieldConstants.TEAM_RED_SPEAKER
                        : Constants.FieldConstants.TEAM_BLUE_SPEAKER;
                break;
            case Amp:
                this.targetRot = DriverStationUtils.getCurrentAlliance() == Alliance.Red
                        ? Constants.FieldConstants.TEAM_RED_AMP
                        : Constants.FieldConstants.TEAM_BLUE_AMP;
                break;
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double spin = 0;
        if (targetPos != null) {
            spin = _autoOrient.calculate(targetPos, _drivetrain.poseFilter.getEstimatedPosition());
        }

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                0,
                0,
                spin,
                true);

        _drivetrain.drive(driveCommandData);
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        double spin = 0;
        if (targetPos != null) {
            spin = _autoOrient.calculate(targetPos, _drivetrain.poseFilter.getEstimatedPosition());
        } else if (targetRot != null) {
            spin = _autoOrient.calculate(targetRot, _drivetrain.poseFilter.getEstimatedPosition());
        }

        SmartDashboard.putNumber("autospin", spin);
        if (Math.abs(spin) <= 0.04) {
            return true;
        } else {
            return false;
        }
    }
}