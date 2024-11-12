package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Acquisition extends KillableSubsystem {
  private Spark acquisitionMotor = new Spark(RobotMap.Acquisition.ACQUISITION_MOTOR_ID);
  private static final double acquisitionDefaultSpeed = Constants.Acquisition.ACQUISITION_SPEED;
  public AcquisitionStates acquisitionState = AcquisitionStates.OFF;

  public Acquisition() {
    toggle(AcquisitionStates.OFF);

    ShuffleboardUI.Overview.setAcquisition(() -> acquisitionMotor.get() != 0);
    ShuffleboardUI.Autonomous.setAcquisition(() -> acquisitionMotor.get() != 0);
    ShuffleboardUI.Test.addMotor("Acquisition", acquisitionMotor);
  }

  public enum AcquisitionStates {
    IN,
    REVERSE,
    OFF;
  }

  public void toggle(AcquisitionStates state, double speed) {
    acquisitionState = state;
    switch (state) {
      case IN: // take in note
        acquisitionMotor.set(speed);
        break;
      case REVERSE: // push out note
        acquisitionMotor.set(-speed);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        acquisitionMotor.set(0);
        break;
    }
  }

  public void toggle(AcquisitionStates state) {
    toggle(state, acquisitionDefaultSpeed);
  }

  @Override
  public void kill() {
    toggle(AcquisitionStates.OFF);
  }
}
