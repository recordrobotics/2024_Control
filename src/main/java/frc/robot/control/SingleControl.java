package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;

public class SingleControl implements IControlInput {

	private Joystick _gamepad;

	private static final double speedModifier = 0.3;

	public SingleControl(int port) {
		_gamepad = new Joystick(port);
	}

	public double getX() {
		// Robot and Joystick axises are flipped
		double input = _gamepad.getY();
		if (input >= 0.1 || input <= -0.1) {
			return input * speedModifier;
		}
		return 0;
	}

	public double getY() {
		// Robot and Joystick axises are flipped
		double input = _gamepad.getX();
		if (input >= 0.1 || input <= -0.1) {
			return input * speedModifier;
		}
		return 0;
	}

	public double setSpin() {
		double input = _gamepad.getTwist();
		if (input >= 0.1 || input <= -0.1)
			return input * speedModifier;
		return 0;
	}

	// TODO functions to get climbers and aquisition 

	public boolean getAcquisition() {
		return _gamepad.getTrigger();
	}

	public boolean toggleClimbers() {
		return _gamepad.getTriggerPressed();
	}

	public boolean spinFlywheel() {
		return _gamepad.getTrigger();
	}
}
