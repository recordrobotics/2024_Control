// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private CANSparkMax flywheelLeft = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID,
            MotorType.kBrushless);
    private CANSparkMax flywheelRight = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID,
            MotorType.kBrushless);

    public Shooter() {
        flywheelLeft.set(0);
        flywheelRight.set(0);
    }

    public void shoot() {
        flywheelLeft.set(0.1);
        flywheelRight.set(0.1);
    }

    public void stop() {
        flywheelLeft.set(0);
        flywheelRight.set(0);
    }
}
