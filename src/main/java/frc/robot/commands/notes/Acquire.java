package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class Acquire extends Command {

  private static Acquisition _acquisition;
  private static Channel _channel;
  private static Photosensor _photosensor;

  public Acquire (Acquisition acquisition, Channel channel, Photosensor photosensor) {
    _acquisition = acquisition;
    _channel = channel;
    _photosensor = photosensor;
    addRequirements(acquisition);
    addRequirements(channel);
    addRequirements(photosensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _acquisition.toggle(AcquisitionStates.IN);
    _channel.toggle(ChannelStates.THROUGH);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _acquisition.toggle(AcquisitionStates.OFF);
    _channel.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _photosensor.getDebouncedValue();
  }
}
