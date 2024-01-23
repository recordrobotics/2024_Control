package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.IControlInput;
import frc.robot.subsystems.Acquisition;

public class ManualAcquisition extends Command{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private Acquisition _acquisition;
    private IControlInput _controls;


     public ManualAcquisition(Acquisition acquisition, IControlInput controls) {
        _acquisition = acquisition;
        _controls = controls;
        addRequirements(acquisition);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_controls.getAcquisition()) _acquisition.run();
    else _acquisition.stop();
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
 