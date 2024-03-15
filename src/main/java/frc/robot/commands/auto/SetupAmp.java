package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;


public class SetupAmp extends SequentialCommandGroup {

  private static Shooter _shooter;
  private static Crashbar _crashbar;

  public SetupAmp (Shooter shooter, Crashbar crashbar, Boolean lowerCrashbar) {
    _shooter = shooter;
    _crashbar = crashbar;

    final Runnable killSpecified = () -> new KillSpecified(_shooter, _crashbar);

    addCommands(
      new InstantCommand(() -> {
            shooter.toggle(ShooterStates.AMP);
            if (lowerCrashbar) {
              crashbar.toggle(CrashbarStates.EXTENDED);
            }
        }).handleInterrupt(killSpecified)
    );
  }
}