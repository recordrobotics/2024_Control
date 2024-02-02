
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Acquisition extends SubsystemBase {
    private Spark lower = new Spark(RobotMap.Aquisition.LOWER_ACQUISTION__MOTOR_ID);
    private Spark upper = new Spark(RobotMap.Aquisition.UPPER_ACQUISITION_MOTOR_ID);
    DigitalInput photosensor = new DigitalInput();

    private static final double LOWER_SPEED = 0.3;
    private static final double UPPER_SPEED = 0.1;

    public Acquisition() {
        toggle(Shooter.ShooterStates.STOP);
    }

    public void toggle(Shooter.ShooterStates state) {
        switch (state) {
            case FLYWHEEL:
                lower.set(0);
                upper.set(UPPER_SPEED);
                break;
            case AQUISITION:
                lower.set(LOWER_SPEED);
                upper.set(UPPER_SPEED);
                break;
            default:
                lower.set(0);
                upper.set(0);
                break;
        }
    }
}
