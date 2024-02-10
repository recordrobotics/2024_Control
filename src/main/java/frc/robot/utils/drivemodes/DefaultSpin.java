package frc.robot.utils.drivemodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.control.DoubleControl;
import frc.robot.utils.DriveCommandData;

public class DefaultSpin {

    public DefaultSpin() {
    }

    /**
     * runs calculations for auto-orient
     * @return
     * DriveCommandData object with drive directions
     */
    public double calculate(DoubleControl _controls) {
        return _controls.getSpin(); 
    }

}
