package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class AcquireSmart extends SequentialCommandGroup {

  /**
   * Command that acquires the note in a way that centers it within the channel.
   *
   * @param acquisition
   * @param channel
   * @param photosensor
   * @param shooter
   */
  public AcquireSmart() {
    final Runnable killSpecified =
        () -> new KillSpecified(Acquisition.instance, Channel.instance, Shooter.instance);

    addCommands(
        // Reverse shooter to ensure note does not come out through the shooter
        new InstantCommand(() -> Shooter.instance.toggle(ShooterStates.REVERSE), Shooter.instance)
            .handleInterrupt(killSpecified),
        // Turns acq and channel on to make the note move into the robot
        new InstantCommand(
                () -> Acquisition.instance.toggle(AcquisitionStates.IN), Acquisition.instance)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> Channel.instance.toggle(ChannelStates.SHOOT), Channel.instance)
            .handleInterrupt(killSpecified),
        // Waits until photosensor
        new WaitUntilCommand(() -> Photosensor.instance.getDebouncedValue()),
        // Turns acq off to prevent more notes from getting acquired
        new InstantCommand(
                () -> Acquisition.instance.toggle(AcquisitionStates.OFF), Acquisition.instance)
            .handleInterrupt(killSpecified),
        // Waits until photosensor off, and then extra 0.15 seconds
        // Wait until note moves fully into the shooter assembly, and then some
        new WaitUntilCommand(() -> !Photosensor.instance.getDebouncedValue())
            .handleInterrupt(killSpecified),
        new WaitCommand(0.15),
        // Turns channel reverse to unsquish the note back to the middle of the channel
        new InstantCommand(() -> Channel.instance.toggle(ChannelStates.REVERSE), Channel.instance)
            .handleInterrupt(killSpecified),
        // Waits until photosensor on, then toggle channel off
        // This stops the note when it is centered in the channel
        new WaitUntilCommand(() -> Photosensor.instance.getDebouncedValue())
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> Channel.instance.toggle(ChannelStates.OFF), Channel.instance)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> Shooter.instance.toggle(ShooterStates.OFF), Shooter.instance)
            .handleInterrupt(killSpecified));
  }
}
// TODO: investigate what happens when interrupted
