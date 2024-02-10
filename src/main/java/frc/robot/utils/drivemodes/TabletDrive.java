// Imports
package frc.robot.utils.drivemodes;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;

//
public class TabletDrive {

    public TabletDrive() {
    }

    /**
     * runs calculations for auto-orient
     * @return
     * DriveCommandData object with drive directions
     */
    public DriveCommandData calculate(DoubleControl _controls, Pose2d swerve_position) {

        // Gets speed level from controller
        double speedLevel = _controls.getSpeedLevel();

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
            _controls.getX() * speedLevel,
            _controls.getY() * speedLevel,
            _controls.getSpin(),
            true);

        // Returns
        return driveCommandData;
    }
}
