
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.RobotMap;

public class Channel extends SubsystemBase {
    private CANSparkMax channelMotor = new CANSparkMax(RobotMap.Channel.CHANNEL_MOTOR_ID, MotorType.kBrushed);
    private DigitalInput photosensor = new DigitalInput(0);

    private static final double acquisitionSpeed = 0.3;

    public Channel() {
        toggle(ChannelStates.OFF);
    }

    public void toggle(ChannelStates state) {
        switch (state) {
            case THROUGH:
                if (!photosensor.get()) {
                    channelMotor.set(acquisitionSpeed);
                }
                else {
                    channelMotor.set(0);
                }
                break;
            case SHOOT:
                if (photosensor.get()) {
                    channelMotor.set(acquisitionSpeed);
                }
                else {
                    channelMotor.set(0);
                }
                break;
            case REVERSE:
                channelMotor.set(-acquisitionSpeed);
                break;
            default:
                channelMotor.set(0);
                break;
        }
    }

    public enum ChannelStates {
        THROUGH,
        SHOOT,
        REVERSE,
        OFF;
    }
}
