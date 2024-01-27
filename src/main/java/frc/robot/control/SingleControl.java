package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class SingleControl implements IControlInput {

	private Joystick _gamepad;

	public SingleControl(int port) {
		_gamepad = new Joystick(port);
	}

	public double getX() {
		// Robot and Joystick axises are flipped
		double input = _gamepad.getY();
		if (input >= Constants.Control.INPUT_X_THRESHOLD || input <= -Constants.Control.INPUT_X_THRESHOLD) {
			return input * Constants.Control.INPUT_SENSITIVITY;
		}
		return 0;
	}

	public double getY() {
		// Robot and Joystick axises are flipped
		double input = _gamepad.getX();
		if (input >= Constants.Control.INPUT_Y_THRESHOLD || input <= -Constants.Control.INPUT_Y_THRESHOLD) {
			return input * Constants.Control.INPUT_SENSITIVITY;
		}
		return 0;
	}

	public double getSpin() {
		double input = _gamepad.getTwist();
		if (input >= Constants.Control.INPUT_SPIN_THRESHOLD || input <= -Constants.Control.INPUT_SPIN_THRESHOLD)
			return input * Constants.Control.INPUT_SENSITIVITY;
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
