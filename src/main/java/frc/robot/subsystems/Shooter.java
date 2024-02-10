// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    private CANSparkMax flywheelL = new CANSparkMax(RobotMap.Shooter.flywheelL, MotorType.kBrushless);
    private CANSparkMax flywheelR = new CANSparkMax(RobotMap.Shooter.flywheelR, MotorType.kBrushless);
    private static final double FLYWHEEL_SPEED = 0.3;

    public Shooter() {
        toggle(ShooterStates.STOP);
    }

    public void toggle(ShooterStates state) {
        switch (state) {
            case FLYWHEEL:
                flywheelL.set(-FLYWHEEL_SPEED);
                flywheelR.set(FLYWHEEL_SPEED);
                break;
            default:
                flywheelL.set(0);
                flywheelR.set(0);
                break;
        }
    }

    public enum ShooterStates {
        FLYWHEEL,
        STOP;
    }
}
