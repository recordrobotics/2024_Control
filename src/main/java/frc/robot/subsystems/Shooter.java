// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private TalonFX flywheel = new TalonFX();

    private static final FLYWHEEL_SPEED = 0.3;

    public Shooter() {
        stop();
    }

    public void shoot(){
        flywheel.set(FLYWHEEL_SPEED);
    }

    public void stop(){
        flywheel.set(0);
    }
    
}
