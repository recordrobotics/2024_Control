package frc.robot.subsystems;

// AHRS Import
import com.kauailabs.navx.frc.AHRS;
// Other imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NavSensor extends SubsystemBase {

	// Creates AHRS _nav object
	private static AHRS _nav = new AHRS(SerialPort.Port.kUSB1);

	// variable to keep track of a reference angle whenever you reset
	private static double referenceAngle;

	public static void initNav() {
		_nav.reset();
		_nav.resetDisplacement();
		referenceAngle = _nav.getAngle();
	}

	// Navsensor Constructor, Create instance
	public NavSensor() {
	}

	// stores the reference angle as whatever the angle is currently measured to be
	public void resetAngleAdjustment() {
		referenceAngle = _nav.getAngle();
	}

	// Gets the angle minus the reference angle
	public Rotation2d getAdjustedAngle() {
		return Rotation2d.fromDegrees(_nav.getAngle() - referenceAngle);
	}

	@Override
	public void periodic() {
		//SmartDashboard.putBoolean("Nav connected", _nav.isConnected());
		//SmartDashboard.putBoolean("Nav Cal", _nav.isCalibrating());
	}

	/*
	 * public double getPitch() {
	 * double pitch = _nav.getRoll();
	 * return Units.degreesToRadians(pitch);
	 * }
	 * 
	 * /*
	 * public double getPitch() {
	 * double pitch = _nav.getRoll();
	 * return Units.degreesToRadians(pitch);
	 * }
	 * 
	 * public double getRoll() {
	 * double roll = _nav.getPitch();
	 * return Units.degreesToRadians(-1*roll);
	 * }
	 * 
	 * public double getYaw() {
	 * double yaw = _nav.getYaw();
	 * return Units.degreesToRadians(-1*yaw);
	 * }
	 * 
	 * //None of the below are guarenteed to work (weird axis changes)
	 * 
	 * public double getDisplacementX() {
	 * return _nav.getDisplacementX();
	 * }
	 * 
	 * public double getDisplacementY() {
	 * return _nav.getDisplacementY();
	 * }
	 * 
	 * public double getDisplacementZ() {
	 * return _nav.getDisplacementZ();
	 * }
	 * 
	 * void sensorResetAngle() {
	 * _nav.reset();
	 * }
	 * 
	 * void resetDisplacement() {
	 * _nav.resetDisplacement();
	 * }
	 * 
	 * void resetAll(){
	 * sensorResetAngle();
	 * resetDisplacement();
	 * 
	 * }
	 */
}