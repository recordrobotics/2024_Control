// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private Spark flywheel = new Spark(RobotMap.Shooter.flywheel);
    private Spark lower = new Spark(RobotMap.Aquisition.lower);
    private Spark upper = new Spark(RobotMap.Aquisition.upper);

    private static final double FLYWHEEL_SPEED = 0.3;
    private static final double LOWER_SPEED = 0.3;
    private static final double UPPER_SPEED = 0.1;

    public Shooter() {
        shoot(ShooterStates.STOP);
    }

    public void shoot(ShooterStates state) {
        switch (state) {
            case FLYWHEEL:
                flywheel.set(FLYWHEEL_SPEED);
                lower.set(0);
                upper.set(0);
                break;
            case AQUISITION:
                flywheel.set(0);
                lower.set(LOWER_SPEED);
                upper.set(UPPER_SPEED);
                break;
            default:
                flywheel.set(0);
                lower.set(0);
                upper.set(0);
                break;
        }
    }

    public enum ShooterStates {
        FLYWHEEL,
        AQUISITION,
        STOP;
    }
}
