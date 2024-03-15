package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KillableSubsystem;


public class KillSpecified extends Command {

    private KillableSubsystem[] _subsytems;
    private Boolean _shouldContinuouslyExecute;

    /**
     * Kills whatever subsystem you put into the system
     * @param subsystems
     * 
     */
    public KillSpecified (KillableSubsystem... subsystems) {
        this(false, subsystems);
    }

    /**
     * 
     * @param execute whether or not the command should run continuously
     * @param subsystems 
     */
    public KillSpecified (Boolean shouldContinuouslyExecute, KillableSubsystem... subsystems) {
        addRequirements(subsystems); //TODO: is this necessary/helpful?
        _subsytems = subsystems;
        _shouldContinuouslyExecute = shouldContinuouslyExecute;
    }

    @Override
    public void initialize() {
        for (KillableSubsystem subsystem: _subsytems) {
            subsystem.kill();
        }
    }

    @Override
    public void execute() {
        for (KillableSubsystem subsystem: _subsytems) {
            subsystem.kill();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return !_shouldContinuouslyExecute;
    }

}
