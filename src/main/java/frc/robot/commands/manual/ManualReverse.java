package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class ManualReverse extends Command {

  public ManualReverse() {
    addRequirements(RobotContainer.acquisition);
    addRequirements(RobotContainer.channel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.acquisition.toggle(AcquisitionStates.REVERSE);
    RobotContainer.channel.toggle(ChannelStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.acquisition.toggle(AcquisitionStates.OFF);
    RobotContainer.channel.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
