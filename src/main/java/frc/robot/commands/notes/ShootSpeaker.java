package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import edu.wpi.first.wpilibj.Timer;

public class ShootSpeaker extends Command {

  private static Channel _channel;
  private static Shooter _shooter;

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 4;
  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 2;

  // Init timer
  protected Timer m_timer = new Timer();

  public ShootSpeaker (Channel channel, Shooter shooter) {
    _channel = channel;
    _shooter = shooter;
    addRequirements(channel);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.toggle(ShooterStates.SPEAKER);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(flywheelSpinupTime) && _channel.channelState == ChannelStates.OFF) {
      _channel.toggle(ChannelStates.SHOOT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.toggle(ShooterStates.OFF);
    _channel.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(flywheelSpinupTime + shootTime);
  }
}
