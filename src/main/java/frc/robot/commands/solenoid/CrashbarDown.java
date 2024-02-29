package frc.robot.commands.solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Crashbar;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrashbarDown extends Command {

  private static Crashbar _crashbar;

  // Init timer
  protected Timer m_timer = new Timer();

  public CrashbarDown (Crashbar crashbar) {
    addRequirements(crashbar);
    _crashbar = crashbar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _crashbar.toggle(CrashbarStates.RETRACTED);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //_crashbar.toggle(CrashbarStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}