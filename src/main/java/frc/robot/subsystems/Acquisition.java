
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.RobotMap;

public class Acquisition extends SubsystemBase {
    private CANSparkMax acquisitionMotor = new CANSparkMax(RobotMap.Acquisition.ACQUISITION_MOTOR_ID, MotorType.kBrushed);
    // DigitalInput photosensor = new DigitalInput(0);

    private static final double acquisitionSpeed = 0.3;

    public Acquisition() {
        toggle(AcquisitionStates.OFF);
    }

    public void toggle(AcquisitionStates state) {
        switch (state) {
            case IN:
                acquisitionMotor.set(acquisitionSpeed);
                break;
            case REVERSE:
                acquisitionMotor.set(-acquisitionSpeed);
                break;
            default:
                acquisitionMotor.set(0);
                break;
        }
    }

    public enum AcquisitionStates {
        IN,
        REVERSE,
        OFF;
    }
}
