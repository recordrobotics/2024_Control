package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants;

public class DoubleControl {

	private Joystick stickpad;
	private Joystick gamepad;

	public DoubleControl(int stickpadPort, int gamepadPort) {
		stickpad = new Joystick(stickpadPort);
		gamepad = new Joystick(gamepadPort);
	}

	public double getX() {
		// Robot and Joystick axises are flipped
		double input = -stickpad.getY();
		if (input >= Constants.Control.INPUT_X_THRESHOLD || input <= -Constants.Control.INPUT_X_THRESHOLD) {
			return input * Constants.Control.INPUT_SENSITIVITY;
		}
		return 0;
	}

	public double getY() {
		// Robot and Joystick axises are flipped
		double input = -stickpad.getX();
		if (input >= Constants.Control.INPUT_Y_THRESHOLD || input <= -Constants.Control.INPUT_Y_THRESHOLD) {
			return input * Constants.Control.INPUT_SENSITIVITY;
		}
		return 0;
	}

	public double getSpin() {
		double input = -stickpad.getTwist();
		if (input >= Constants.Control.INPUT_SPIN_THRESHOLD || input <= -Constants.Control.INPUT_SPIN_THRESHOLD)
			return Constants.RemapAbsoluteValue(input, Constants.Control.SPIN_INPUT_REMAP_LOW,
					Constants.Control.SPIN_INPUT_REMAP_HIGH) * Constants.Control.SPIN_INPUT_SENSITIVITY;
		return 0;
	}

	public boolean getResetPressed() {
		return stickpad.getRawButtonPressed(2);
	}

	public boolean getAutoOrientChain() {
		return stickpad.getRawButton(7);
	}

	public boolean getAutoOrientSpeaker() {
		return stickpad.getRawButton(9) || gamepad.getRawButton(3);
	}

	public boolean getAutoOrientAmp() {
		return stickpad.getRawButton(11) || gamepad.getRawButton(4);
	}

	public boolean getAutoChain() {
		return stickpad.getRawButtonPressed(8);
	}

	public boolean getAutoScoreSpeaker() {
		return stickpad.getRawButtonPressed(10) || gamepad.getRawButtonPressed(1);
	}

	public boolean getAutoScoreAmp() {
		return stickpad.getRawButtonPressed(12) || gamepad.getRawButtonPressed(2);
	}

	/**
	 * Takes speedlevel slider on control input and remaps from -1-->1 to 0.5-->2
	 * TODO: add to constants
	 * @return
	 * Speedlevel control from 0.5 --> 2
	 */
	public double getSpeedLevel() {
		// Remap -1 --> 1 to 0 --> 1
		double remap1 = (-stickpad.getRawAxis(3) + 1.0) / 2.0;
		// Remap 0 --> 1 to 0.5 --> 2
		double remap2 = remap1 * (2 - 0.5) + 0.5;
		// Returns
		return remap2;
	}

	public boolean getAcquireNormal() {
		return gamepad.getRawAxis(2) > 0.3;
	}

	public boolean getAcquireReverse() {
		return gamepad.getRawButton(5);
	}

	public boolean getShoot() {
		return gamepad.getRawAxis(3) > 0.3 || gamepad.getRawButton(6);
	}

	public boolean getChainUp() {
		return gamepad.getRawAxis(1) < -0.5;
	}

	public boolean getChainDown() {
		return gamepad.getRawAxis(1) > 0.5;
	}

	public boolean getKillAuto() {
		return gamepad.getRawButton(8);
	}
}
