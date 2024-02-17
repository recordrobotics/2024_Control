package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class DoubleControl {

	// Sets ups controller classes
	private Joystick stickpad;
	private XboxController xbox_controller;

	private JoystickOrientation joystickOrientation = JoystickOrientation.XAxisTowardsTrigger;

	// Constructor
	public DoubleControl(int stickpadPort, int gamepadPort) {
		stickpad = new Joystick(stickpadPort);
		xbox_controller = new XboxController(gamepadPort);
	}

	public void setJoystickOrientation(JoystickOrientation joystickOrientation) {
		this.joystickOrientation = joystickOrientation;
	}

	/**
	 * TODO: getX and getY currently do not subtract threshold from the final value.
	 * I think we should subtract threshold. We should talk about changing this.
	 */

	/**
	 * @return remapped joystick value x horizontal (sets a min threshold,
	 *         multiplies by input sens)
	 */
	public double getX() {

		// Gets raw value
		double input;
		switch (joystickOrientation) {
			case XAxisTowardsTrigger:
				input = -stickpad.getY();
				break;
			case YAxisTowardsTrigger:
				input = stickpad.getX();
				break;
			default:
				input = 0;
				break;
		}
		// Gets whether or not the spin input is negative or positive
		double subtract_threshold = Math.max(0, Math.abs(input) - Constants.Control.JOYSTICK_X_THRESHOLD); // How much
																											// the input
																											// is above
																											// the
																											// threshold
																											// (absolute
																											// value)
		double proportion = subtract_threshold / (1 - Constants.Control.JOYSTICK_X_THRESHOLD); // What proportion
																								// (threshold to value)
																								// is of (threshold to
																								// 1)
		// Multiplies by spin sensitivity and returns
		double final_x = Math.signum(input) * proportion * Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY;
		return final_x;

	}

	/**
	 * @return remapped joystick y value (sets a min threshold, multiplies by input
	 *         sens)
	 */
	public double getY() {

		// Gets raw value
		double input;
		switch (joystickOrientation) {
			case XAxisTowardsTrigger:
				input = -stickpad.getX();
				break;
			case YAxisTowardsTrigger:
				input = -stickpad.getY();
				break;
			default:
				input = 0;
				break;
		}
		// Gets whether or not the spin input is negative or positive
		double subtract_threshold = Math.max(0, Math.abs(input) - Constants.Control.JOSYTICK_Y_THRESHOLD); // How much
																											// the input
																											// is above
																											// the
																											// threshold
																											// (absolute
																											// value)
		double proportion = subtract_threshold / (1 - Constants.Control.JOSYTICK_Y_THRESHOLD); // What proportion
																								// (threshold to value)
																								// is of (threshold to
																								// 1)
		// Multiplies by spin sensitivity and returns
		double final_y = Math.signum(input) * proportion * Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY;
		return final_y;

	}

	/**
	 * @return remapped joystick spin value (sets a min threshold, subtracts
	 *         threshold, multiplied by input sens)
	 */
	public double getSpin() {

		// Gets raw twist value
		double input = -stickpad.getTwist();
		// Gets whether or not the spin input is negative or positive
		double sign = Math.signum(input);
		// How much the input is above the threshold (absolute value)
		double subtract_threshold = Math.max(0, Math.abs(input) - Constants.Control.JOYSTICK_SPIN_THRESHOLD);
		// What proportion (threshold to value) is of (threshold to 1)
		double proportion = subtract_threshold / (1 - Constants.Control.JOYSTICK_SPIN_THRESHOLD);
		// Multiplies by spin sensitivity
		double final_spin = proportion * sign * Constants.Control.JOYSTICK_SPIN_SENSITIVITY;

		// Returns
		return final_spin;

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
	 * TODO: add to constants, make it not insanely fast
	 * 
	 * @return
	 *         Speedlevel control from 0.5 --> 2
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

	public boolean getCrashbarExtend() {
		return xbox_controller.getRawAxis(5) < -0.5;
	}

	public boolean getCrashbarRetract() {
		return xbox_controller.getRawAxis(5) > 0.5;
	}

	public boolean getKillAuto() {
		return xbox_controller.getRawButton(8);
	}

	/**
	 * TABLET COMMANDS
	 */

	/**
	 * Converts (-1, 1) to (0,1)
	 * 
	 * @return x coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletX() {
		double x = (xbox_controller.getRawAxis(0) + 1) / 2;
		return x;
	}

	/**
	 * Converts (-1, 1) to (0,1)
	 * 
	 * @return y coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletY() {
		double y = (xbox_controller.getRawAxis(1) + 1) / 2;
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
	}
}
