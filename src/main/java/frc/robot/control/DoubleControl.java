package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class DoubleControl {

	private Joystick stickpad;
	private XboxController xbox_controller;

	public DoubleControl(int stickpadPort, int gamepadPort) {
		stickpad = new Joystick(stickpadPort);
		xbox_controller = new XboxController(gamepadPort);
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
		return stickpad.getRawButton(9) || xbox_controller.getRawButton(3);
	}

	public boolean getAutoOrientAmp() {
		return stickpad.getRawButton(11) || xbox_controller.getRawButton(4);
	}

	public boolean getAutoChain() {
		return stickpad.getRawButtonPressed(8);
	}

	public boolean getAutoScoreSpeaker() {
		return stickpad.getRawButtonPressed(10) || xbox_controller.getRawButtonPressed(1);
	}

	public boolean getAutoScoreAmp() {
		return stickpad.getRawButtonPressed(12) || xbox_controller.getRawButtonPressed(2);
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
		return xbox_controller.getRawAxis(2) > 0.3;
	}

	public boolean getAcquireReverse() {
		return xbox_controller.getRawButton(5);
	}

	public boolean getShoot() {
		return xbox_controller.getRawAxis(3) > 0.3 || xbox_controller.getRawButton(6);
	}

	public boolean getChainUp() {
		return xbox_controller.getRawAxis(1) < -0.5;
	}

	public boolean getChainDown() {
		return xbox_controller.getRawAxis(1) > 0.5;
	}

	public boolean getKillAuto() {
		return xbox_controller.getRawButton(8);
	}


	/**
	 * TABLET COMMANDS
	 */

	/**
	 * @return x coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletX() {
		double x = (xbox_controller.getRawAxis(0) + 1)/2;
		return x;
	}

	/**
	 * @return y coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletY() {
		double y = (xbox_controller.getRawAxis(1) + 1)/2;
		return y;
	}

	/**
	 * @return whether or not tablet is pressed down
	 */
	public double getTabletPressure() {

		// Checks that the Z axis (height of the pen) is below a certain amount 
		// (If it isn't, the pressure reading is probably a hardware error)
		if (xbox_controller.getRawAxis(4) > 0.03 && xbox_controller.getRawAxis(4) < 0.25) {
			return -1 * xbox_controller.getRawAxis(5);
		}
		return 0;
		/*
		double pressure = -1 * xbox_controller.getRawAxis(5); // Makes pressure positive (0 --> 1)
		return pressure > 0.1; // If the pressure > 0.1, returns  */
	}
}
