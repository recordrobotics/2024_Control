package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subroutines.PushSpeaker;
import frc.robot.commands.subroutines.SetupSpeaker;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends SequentialCommandGroup {

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.5; // 1.5;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 2;

  /**
   * Command that shoots the note into the speaker. Manages all relevant subsystems to do so.
   *
   * @param channel
   * @param shooter
   */
  public ShootSpeaker() {
    addRequirements(Channel.instance);
    addRequirements(Shooter.instance);

    addCommands(
        new SetupSpeaker(Shooter.instance),
        new WaitCommand(flywheelSpinupTime),
        new PushSpeaker(Channel.instance, Shooter.instance, shootTime));
  }
}
