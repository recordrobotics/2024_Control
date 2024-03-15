package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardUI;

public class Acquisition extends KillableSubsystem {
    private Spark acquisitionMotor = new Spark(RobotMap.Acquisition.ACQUISITION_MOTOR_ID);
    private static final double acquisitionDefaultSpeed = Constants.Acquisition.ACQUISITION_SPEED;
    public AcquisitionStates acquisitionState = AcquisitionStates.OFF;

    public Acquisition() {
        toggle(AcquisitionStates.OFF);
        setupShuffleboard();
    }

    public enum AcquisitionStates {
        IN,
        REVERSE,
        OFF;
    }

    public void toggle(AcquisitionStates state, double speed) {
        acquisitionState = state;
        switch (state) {
            case IN:
                acquisitionMotor.set(speed);
                break;
            case REVERSE:
                acquisitionMotor.set(-speed);
                break;
            default:
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

    private void setupShuffleboard() {
        ShuffleboardUI.Overview.getTab().addBoolean("Acquisition", ()->acquisitionMotor.get()!=0)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(7, 0)
            .withSize(1, 1);

        ShuffleboardUI.Autonomous.getTab().addBoolean("Acquisition", ()->acquisitionMotor.get()!=0)
        	.withWidget(BuiltInWidgets.kBooleanBox)
        	.withPosition(9, 0)
        	.withSize(1, 1);

        ShuffleboardUI.Test.getTab().add("Acquisition", acquisitionMotor)
            .withWidget(BuiltInWidgets.kMotorController);
    }
}
