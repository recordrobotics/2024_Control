package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import edu.wpi.first.wpilibj.Timer;

public class ShootAmp extends Command {

  private static Channel _channel;
  private static Shooter _shooter;
  private static Crashbar _crashbar;

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 3;
  private final double crashbarExtendTime = 3;
  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 2;

  // Init timer
  protected Timer m_timer = new Timer();

  public ShootAmp (Channel channel, Shooter shooter, Crashbar crashbar) {
    _channel = channel;
    _shooter = shooter;
    _crashbar = crashbar;
    addRequirements(channel);
    addRequirements(shooter);
    addRequirements(crashbar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.toggle(ShooterStates.SPEAKER);
    _crashbar.toggle(CrashbarStates.EXTENDED);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(flywheelSpinupTime) && m_timer.hasElapsed(crashbarExtendTime) && _channel.channelState == ChannelStates.OFF) {
      _channel.toggle(ChannelStates.SHOOT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.toggle(ShooterStates.OFF);
    _channel.toggle(ChannelStates.OFF);
    _crashbar.toggle(CrashbarStates.RETRACTED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(flywheelSpinupTime + shootTime);
  }
}
