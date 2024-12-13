package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;

public class ManualChannel extends Command {

  public ManualChannel() {
    addRequirements(Channel.instance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Channel.instance.toggle(ChannelStates.SHOOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Channel.instance.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
