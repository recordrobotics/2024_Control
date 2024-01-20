package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.control.IControlInput;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends CommandBase{

    private CShooter _shooter;
    private IControlInput _controls;

     public ManualClimbers(Shooter shooter, IControlInput controls) {
        _shooter = shooter;
        _controls = controls;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_controls.spinFlywheel()) _shooter.shoot();
    else _shooter.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
