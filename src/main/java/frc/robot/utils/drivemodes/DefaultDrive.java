// Imports
package frc.robot.utils.drivemodes;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;

//
public class DefaultDrive {

    public DefaultDrive() {
    }

    /**
     * runs calculations for auto-orient
     * @return
     * DriveCommandData object with drive directions
     */
    public DriveCommandData calculate(DoubleControl _controls, double spin, Pose2d swerve_position, boolean field_relative) {

        // Gets speed level from controller
        double speedLevel = _controls.getSpeedLevel();

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
            _controls.getX() * speedLevel,
            _controls.getY() * speedLevel,
            spin,
            field_relative);

        // Returns
        return driveCommandData;
    }
}
