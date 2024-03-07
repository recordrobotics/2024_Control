
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Channel extends SubsystemBase {
    private Spark channelMotor = new Spark(RobotMap.Channel.CHANNEL_MOTOR_ID);
    private static final double channelDefaultSpeed = Constants.Channel.CHANNEL_SPEED;

    public Channel() {
        toggle(ChannelStates.OFF);
    }

    public void toggle(ChannelStates state, double speed) {
        switch (state) {
            case THROUGH:
                channelMotor.set(speed);
                break;
            case SHOOT:
                channelMotor.set(speed);
                break;
            case REVERSE:
                channelMotor.set(-speed);
                break;
            default:
                channelMotor.set(0);
                break;
        }
    }

    public void toggle(ChannelStates state) {
        toggle(state, channelDefaultSpeed);
    }

    public enum ChannelStates {
        THROUGH,
        SHOOT,
        REVERSE,
        OFF;
    }
}
