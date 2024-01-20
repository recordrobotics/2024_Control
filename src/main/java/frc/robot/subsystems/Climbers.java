
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase{

    private Solenoid left = new Solenoid(CTREPCM, RobotMap.Climbers.left);
    private Solenoid right = new Solenoid(CTREPCM, RobotMap.Climbers.right);

    public Climbers() {
        left.close();
        right.close();
    }

    void toggle(){
        left.toggle();
        right.toggle();
    }
    
}
