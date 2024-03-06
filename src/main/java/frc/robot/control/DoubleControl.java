package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.SimpleMath;

public class DoubleControl {

	// Sets ups controller classes
	private Joystick stickpad;
	private XboxController xbox_controller;
	private GenericHID extra_controller;

	private JoystickOrientation joystickOrientation = JoystickOrientation.XAxisTowardsTrigger;

	// Constructor
	public DoubleControl(int stickpadPort, int xboxPort, int extraControllerPort) {
		stickpad = new Joystick(stickpadPort);
		xbox_controller = new XboxController(xboxPort);
		if (extraControllerPort != -1) {
			extra_controller = new GenericHID(extraControllerPort);
		}
	}

	public DoubleControl(int stickpadPort, int xboxPort) {
		this(stickpadPort, xboxPort, -1);
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
	 *         AWAY FROM DRIVER!!!!
	 */
	public double getX() {

		// Gets raw value
		double input;
		switch (joystickOrientation) {
			case XAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -stickpad.getY();
				else
					input = stickpad.getY();
				break;
			case YAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = stickpad.getX();
				else
					input = -stickpad.getX();
				break;
			default:
				input = 0;
				break;
		}

		// How much the input is above the threshold (absolute value)
		double subtract_threshold = Math.max(0, Math.abs(input) - Constants.Control.JOYSTICK_X_THRESHOLD);

		// What proportion (threshold to value) is of (threshold to 1)
		double proportion = subtract_threshold / (1 - Constants.Control.JOYSTICK_X_THRESHOLD);

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
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -stickpad.getX();
				else
					input = stickpad.getX();
				break;
			case YAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -stickpad.getY();
				else
					input = stickpad.getY();
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
		double proportion = subtract_threshold / (1 - Constants.Control.JOSYTICK_Y_THRESHOLD);
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
		return stickpad.getRawButton(9);
	}

	public boolean getAutoOrientAmp() {
		return stickpad.getRawButton(11);
	}

	public boolean getAutoChain() {
		return stickpad.getRawButtonPressed(8);
	}

	public boolean getAutoScoreSpeaker() {
		return stickpad.getRawButtonPressed(10);
	}

	public boolean getAutoAcquireCorner() {
		return stickpad.getRawButtonPressed(12);
	}

	public boolean getAutoScoreAmp() {
		return stickpad.getRawButtonPressed(12);
	}

	public boolean getTeleAutoStart() {
		return stickpad.getRawButtonPressed(3);
	}

	public Pair<Rotation2d, Boolean> getAngleDPad() {
		double pov = xbox_controller.getPOV();
		if (pov == -1) {
			return new Pair<Rotation2d, Boolean>(new Rotation2d(), false);
		} else {
			return new Pair<Rotation2d, Boolean>(
					SimpleMath.JoystickToFieldPolar(joystickOrientation, Rotation2d.fromDegrees(pov)), true);
		}
	}

	/**
	 * Takes speedlevel slider on control input and remaps to 1.4-->7.0
	 * 
	 * @return
	 *         Speedlevel control from 1.4 --> 7.0
	 */
	public double getDirectionalSpeedLevel() {
		// Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
		return SimpleMath.Remap(stickpad.getRawAxis(3), 1, -1, Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
				Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);
	}

	public double getSpinSpeedLevel() {
		// Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
		return SimpleMath.Remap(stickpad.getRawAxis(3), 1, -1, Constants.Control.SPIN_SPEED_METER_LOW,
				Constants.Control.SPIN_SPEED_METER_HIGH);
	}

	public boolean getAcquire() {
		return xbox_controller.getLeftTriggerAxis() > 0.3 || stickpad.getRawButton(3); // aka the left trigger axis
	}

	public boolean getReverse() {
		return xbox_controller.getLeftBumper() || stickpad.getRawButton(5); // aka the left trigger button
	}

	public boolean getShootSpeaker() {
		return xbox_controller.getRightTriggerAxis() > 0.3 || stickpad.getRawButton(4); // aka the right trigger axis
	}

	public boolean getShootAmp() {
		return xbox_controller.getRightBumper() || stickpad.getRawButton(6); // aka the right trigger button
	}

	public boolean getClimberToggle() {
		return xbox_controller.getRawButton(7);
	}

	public boolean getKillAuto() {
		return xbox_controller.getRawButton(8);
	}

	public boolean getManualShootSpeaker() {
		return xbox_controller.getAButton();
	}

	public boolean getManualShootAmp() {
		return xbox_controller.getBButton();
	}

	public boolean getManualAcquisition() {
		return xbox_controller.getXButton();
	}

	public boolean getManualCrashbar() {
		return xbox_controller.getYButton();
	}

	/**
	 * Converts (-1, 1) to (0,1)
	 * 
	 * @return x coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletX() {
		double x = (extra_controller.getRawAxis(0) + 1) / 2;
		return x;
	}

	/**
	 * Converts (-1, 1) to (0,1)
	 * 
	 * @return y coordinate of the tablet (from 0 to 1)
	 */
	public double getTabletY() {
		double y = (extra_controller.getRawAxis(1) + 1) / 2;
		return y;
	}

	/**
	 * @return whether or not tablet is pressed down
	 */
	public double getTabletPressure() {

		// Checks that the Z axis (height of the pen) is below a certain amount
		// (If it isn't, the pressure reading is probably a hardware error)
		if (extra_controller.getRawAxis(4) > 0.03 && extra_controller.getRawAxis(4) < 0.25) {
			return -1 * extra_controller.getRawAxis(5);
		}
		return 0;
	}

	/**
	 * Converts (-1, 1) to (-pi, pi)
	 * 
	 * @return angle of the spin knob (from -pi to pi)
	 */
	public double getSpinKnob() {
		return SimpleMath.Remap(extra_controller.getRawAxis(0), -1, 1, -Math.PI, Math.PI);
	}

	/**
	 * @return
	 *         Whether the spin lock button has gone from being not pressed to
	 *         pressed (one time)
	 */
	public boolean getSpinLockSet() {
		return stickpad.getRawButtonPressed(1);
	}

	/**
	 * @return
	 *         Whether the spinlock button is currently held down
	 */
	public boolean getSpinLockPressed() {
		return stickpad.getRawButton(1);
	}

	public double getXboxDriveX() {

		// Gets raw value
		double input;
		switch (joystickOrientation) {
			case XAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -xbox_controller.getRawAxis(1);
				else
					input = xbox_controller.getRawAxis(1);
				break;
			case YAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = xbox_controller.getRawAxis(0);
				else
					input = -xbox_controller.getRawAxis(0);
				break;
			default:
				input = 0;
				break;
		}

		// How much the input is above the threshold (absolute value)
		double subtract_threshold = Math.max(0, Math.abs(input) - Constants.Control.JOYSTICK_X_THRESHOLD);

		// What proportion (threshold to value) is of (threshold to 1)
		double proportion = subtract_threshold / (1 - Constants.Control.JOYSTICK_X_THRESHOLD);

		// Multiplies by spin sensitivity and returns
		double final_x = Math.signum(input) * proportion * Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY;
		return final_x;

	}

	/**
	 * @return remapped joystick y value (sets a min threshold, multiplies by input
	 *         sens)
	 */
	public double getXboxDriveY() {

		// Gets raw value
		double input;
		switch (joystickOrientation) {
			case XAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -xbox_controller.getRawAxis(0);
				else
					input = xbox_controller.getRawAxis(0);
				break;
			case YAxisTowardsTrigger:
				if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
					input = -xbox_controller.getRawAxis(1);
				else
					input = xbox_controller.getRawAxis(1);
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
		double proportion = subtract_threshold / (1 - Constants.Control.JOSYTICK_Y_THRESHOLD);
		// Multiplies by spin sensitivity and returns
		double final_y = Math.signum(input) * proportion * Constants.Control.JOSYSTICK_DIRECTIONAL_SENSITIVITY;
		return final_y;
	}

	/**
	 * TODO: documentation
	 * 
	 * @return
	 */
	public Pair<Double, Double> getXboxSpinAngle() {
		double MAGNITUDE_THRESHOLD = 0.5;
		// Gets x and y axis of xbox
		double x_axis = xbox_controller.getRawAxis(4);
		double y_axis = xbox_controller.getRawAxis(5);
		// Gets magnitude of xbox axis
		double magnitude = Math.sqrt(x_axis * x_axis + y_axis * y_axis);

		// How much the input is above the threshold (absolute value)
		double subtract_threshold = Math.max(0, magnitude - MAGNITUDE_THRESHOLD);
		// What proportion (threshold to value) is of (threshold to 1)
		double proportion = subtract_threshold / (1 - MAGNITUDE_THRESHOLD);

		double angle = -Math.atan2(y_axis, x_axis);

		double adjusted_angle = SimpleMath.JoystickToFieldPolar(joystickOrientation, angle);

		return new Pair<Double, Double>(adjusted_angle, proportion);
	}

}