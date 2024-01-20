
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Acquisition extends SubsystemBase{

    private TalonFX lower = new TalonFX(RobotMap.Aquisition.lower);
    private TalonFX upper = new TalonFX(RobotMap.Aquisition.upper);

    public Acquisition() {
        stop();
    }

    // TODO change to correct speeds
    public void run() {
        lower.set(0.3);
        upper.set(0.1);
    }

    public void stop() {
        lower.set(0);
        upper.set(0);
    }
    
}
