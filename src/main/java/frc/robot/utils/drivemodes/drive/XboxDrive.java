// Imports
package frc.robot.utils.drivemodes.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;

public class XboxDrive {

    /**
     * runs calculations for auto-orient
     * 
     * @return
     *         DriveCommandData object with drive directions
     */
    public static DriveCommandData calculate(DoubleControl _controls, double spin, Pose2d swerve_position,
            boolean field_relative) {

        // Gets speed level from controller
        double speedLevel = _controls.getDirectionalSpeedLevel();

        SmartDashboard.putNumber("CONTROLLER SPEED", speedLevel);

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                _controls.getXboxDriveX() * speedLevel,
                _controls.getXboxDriveY() * speedLevel,
                spin,
                field_relative);

        // Returns
        return driveCommandData;
    }
}