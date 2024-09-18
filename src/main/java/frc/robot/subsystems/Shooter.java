// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends KillableSubsystem {

    private CANSparkMax flywheelL = new CANSparkMax( // initialize left flywheel
        RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID,
        MotorType.kBrushless);
    private CANSparkMax flywheelR = new CANSparkMax( // initialize right flywheel
        RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID,
        MotorType.kBrushless);

    public Shooter() {
        toggle(ShooterStates.OFF); // initialize as off
        ShuffleboardUI.Test
            .addSlider("Flywheel Left", flywheelL.get(), -1, 1) // LEFT set slider to show value between -1 and 1
            .subscribe(flywheelL::set); // LEFT if the slider is moved, call flywheelL.set
        ShuffleboardUI.Test
            .addSlider("Flywheel Right", flywheelR.get(), -1, 1) // RIGHT set slider to show value between -1 and 1
            .subscribe(flywheelR::set); // RIGHT if the slider is moved, call flywheelR.set
    }

    public enum ShooterStates {
        SPEAKER, // High speed for speaker
        AMP, // Low speed for amp
        REVERSE, // Reverse for note acquisition
        OFF; // Off
    }

    /**
     * Set the current shooter speed to speedL and speedR
     */
    public void toggle(double speedL, double speedR) {
        flywheelL.set(-speedL); // left side is inverted because it is facing the other way
        flywheelR.set(speedR);
    }

    /**
     * Set the current shooter speed on both wheels to speed
     */
    public void toggle(double speed) {
        toggle(speed, speed);
    }

    /**
     * Set the shooter speed to the preset ShooterStates state
     */
    public void toggle(ShooterStates state) {
        switch (state) {
            case SPEAKER: // High speed for speaker
                toggle(Constants.Shooter.SPEAKER_SPEED);
                break;
            case AMP: // Low speed for amp
                toggle(Constants.Shooter.AMP_SPEED);
                break;
            case REVERSE: // Reverse for note acquisition
                toggle(Constants.Shooter.REVERSE_SPEED);
                break;
            case Off: // Off
            default: // should never happen
                toggle(0); // set speed to 0
                break;
        }
    }

    @Override
    public void kill() {
        toggle(ShooterStates.OFF);
    }
}
