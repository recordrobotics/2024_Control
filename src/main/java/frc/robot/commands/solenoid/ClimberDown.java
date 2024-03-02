package frc.robot.commands.solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers.ClimberStates;
import frc.robot.subsystems.Climbers;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberDown extends Command {

  private static Climbers _climbers;


  public ClimberDown (Climbers climbers) {
    addRequirements(climbers);
    _climbers = climbers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _climbers.toggle(ClimberStates.DOWN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _climbers.toggle(ClimberStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}