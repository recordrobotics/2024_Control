package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;

public class ManualReverse extends Command {

  public ManualReverse() {
    addRequirements(Acquisition.instance);
    addRequirements(Channel.instance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Acquisition.instance.toggle(AcquisitionStates.REVERSE);
    Channel.instance.toggle(ChannelStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Acquisition.instance.toggle(AcquisitionStates.OFF);
    Channel.instance.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
