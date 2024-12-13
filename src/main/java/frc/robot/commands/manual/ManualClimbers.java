package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Climbers.ClimberStates;

public class ManualClimbers extends Command {

  public ManualClimbers() {
    addRequirements(Climbers.instance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climbers.instance.toggle(ClimberStates.UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climbers.instance.toggle(ClimberStates.DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
