package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class AcquireSmart extends SequentialCommandGroup {

  private static Acquisition _acquisition;
  private static Channel _channel;
  private static Photosensor _photosensor;

  public AcquireSmart (Acquisition acquisition, Channel channel, Photosensor photosensor) {
    _acquisition = acquisition;
    _channel = channel;
    _photosensor = photosensor;
    addRequirements(acquisition);
    addRequirements(channel);
    addRequirements(photosensor);

    final Runnable killSpecified = () -> new KillSpecified(_acquisition, _channel, _photosensor);

    addCommands(
      // Turns acq on
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.IN), _acquisition).handleInterrupt(killSpecified),
      new InstantCommand(() -> _channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
      // Waits until photosensor
      new WaitUntilCommand(()->_photosensor.getDebouncedValue()),
      // Turns acq off
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.OFF), _acquisition).handleInterrupt(killSpecified),
      // waits until photosensor off, extra time
      new WaitUntilCommand(()->!_photosensor.getDebouncedValue()).handleInterrupt(killSpecified),
      new WaitUntilCommand(0.5),
      // Turns channel reverse
      new InstantCommand(() -> _channel.toggle(ChannelStates.REVERSE), _channel).handleInterrupt(killSpecified),
      // Waits until photosensor on, then toggle channel off
      new WaitUntilCommand(()->_photosensor.getDebouncedValue()).handleInterrupt(killSpecified),
      new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified)
    );
  }
}