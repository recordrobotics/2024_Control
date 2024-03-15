package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardUI;

public class Channel extends KillableSubsystem {
    private Spark channelMotor = new Spark(RobotMap.Channel.CHANNEL_MOTOR_ID);

    public Channel() {
        toggle(ChannelStates.OFF);
        setupShuffleboard();
    }

    public enum ChannelStates {
        THROUGH,
        SHOOT,
        REVERSE,
        OFF;
    }

    public void toggle(double speed) {
        channelMotor.set(speed);
    }

    public void toggle(ChannelStates state) {
        switch (state) {
            case THROUGH:
                toggle(Constants.Channel.THROUGH_SPEED);
                break;
            case SHOOT:
                toggle(Constants.Channel.SHOOT_SPEED);
                break;
            case REVERSE:
                toggle(Constants.Channel.REVERSE_SPEED);
                break;
            default:
                toggle(0);
                break;
        }
    }

    @Override
    public void kill() {
        toggle(ChannelStates.OFF);
    }

    private void setupShuffleboard() {
        var widget = ShuffleboardUI.Test.getTab().add("Channel", channelMotor);
        widget.withWidget(BuiltInWidgets.kMotorController);
    }
    
}