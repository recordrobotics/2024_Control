package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subroutines.PushAmp;
import frc.robot.commands.subroutines.SetupAmp;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends SequentialCommandGroup {

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.3;

  private final double crashbarExtendTime = 0.4;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 0.7;

  /**
   * Command that shoots the note into the amp. Manages all relevant subsystems to do so (including
   * lowering crashbar, waiting for the shooter to speed up, etc)
   *
   * @param channel
   * @param shooter
   */
  public ShootAmp() {
    addRequirements(Channel.instance);
    addRequirements(Shooter.instance);
    addRequirements(Crashbar.instance);

    addCommands(
        new SetupAmp(Shooter.instance, Crashbar.instance, true),
        new WaitCommand(Math.max(flywheelSpinupTime, crashbarExtendTime)),
        new PushAmp(Channel.instance, Shooter.instance, Crashbar.instance, shootTime));
  }
}
