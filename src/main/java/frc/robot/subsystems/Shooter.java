// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private Spark flywheel = new Spark(RobotMap.Shooter.flywheel);
    private Acquisition acquisition = new Acquisition();
    private static final double FLYWHEEL_SPEED = 0.3;

    public Shooter() {
        shoot(ShooterStates.STOP);
    }

    public void shoot(ShooterStates state) {
        switch (state) {
            case FLYWHEEL:
                flywheel.set(FLYWHEEL_SPEED);
                acquisition.toggle(state);
                break;
            case AQUISITION:
                flywheel.set(0);
                acquisition.toggle(state);
                break;
            default:
                flywheel.set(0);
                acquisition.toggle(state);
                break;
        }
    }

    public enum ShooterStates {
        FLYWHEEL,
        AQUISITION,
        STOP;
    }
}
