// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Climbers.ClimberStates;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RobotKill extends Command {
  private Drivetrain _drivetrain;
  private Acquisition _acquisition;
  private Channel _channel;
  private Climbers _climbers;
  private Shooter _shooter;
  private Crashbar _crashbar;

  

  public RobotKill(Drivetrain drivetrain, 
                   Acquisition acquisition, 
                   Channel channel, 
                   Climbers climbers, 
                   Shooter shooter, 
                   Crashbar crashbar) {
    _drivetrain = drivetrain;
    _acquisition = acquisition;
    _channel = channel;
    _climbers = climbers;
    _shooter = shooter;
    _crashbar = crashbar;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute () {
    _drivetrain.stop();
    _acquisition.toggle(AcquisitionStates.OFF);
    _channel.toggle(ChannelStates.OFF);
    _climbers.toggle(ClimberStates.OFF);
    _shooter.toggle(ShooterStates.OFF);
    _crashbar.toggle(CrashbarStates.OFF);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}