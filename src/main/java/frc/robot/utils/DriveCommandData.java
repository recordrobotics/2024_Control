package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.control.DoubleControl;

/**
 * An object that contains all relevant information for the drivetrain to drive
 */
public class DriveCommandData {

        // Variable definitions
        public double xSpeed;
        public double ySpeed;
        public double rot;
        public boolean fieldRelative;

        // Constructor for an object that contains all relevant information for the
        // drivetrain to drive */
        /**
         * Struct to hold data that will be read by {@link frc.robot.subsystems.Drivetrain}
         * @param xSpeed
         * @param ySpeed
         * @param rot
         * @param fieldRelative
         */
        public DriveCommandData(
                        double xSpeed,
                        double ySpeed,
                        double rot,
                        boolean fieldRelative) {
                this.xSpeed = xSpeed;
                this.ySpeed = ySpeed;
                this.rot = rot;
                this.fieldRelative = fieldRelative;

        }

        /**
         * Finds how fast the robot should be moving given tablet pressure
         * 
         * @return
         *         tablet pressure in m/s
         */
        private double speedFromPressure(double tablet_pressure) {

                double PRESSURE_THRESHOLD = 0.2;
                double MIN_SPEED = 0.2;
                double STEEPNESS = 2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling

                if (tablet_pressure < PRESSURE_THRESHOLD) {
                        return 0;
                }

                else {
                        double coeff_1 = Math.pow((tablet_pressure - PRESSURE_THRESHOLD) / (1 - PRESSURE_THRESHOLD),
                                        STEEPNESS);
                        double coeff_2 = 1 - MIN_SPEED;
                        double final_speed = coeff_1 * coeff_2 + MIN_SPEED;
                        return final_speed;
                }
        }

        // TODO: find a way to put the constants here in Constants.java
        /**
         * Tablet Drive
         * runs calculations for auto-orient
         * 
         * @return
         *         DriveCommandData object with drive directions
         */
        public void calculate(DoubleControl _controls, double spin, Pose2d swerve_position,
                        Field2d m_field) {

                // Puts raw tablet data on Smartdashboard
                SmartDashboard.putNumber("pressure", _controls.getTabletPressure());
                SmartDashboard.putNumber("tablet x", _controls.getTabletX());
                SmartDashboard.putNumber("tablet y", _controls.getTabletY());

                // Gets target speed, x, and y
                double speed = speedFromPressure(_controls.getTabletPressure());
                double target_x = _controls.getTabletX() * Constants.FieldConstants.FIELD_X_DIMENSION;
                double target_y = _controls.getTabletY() * Constants.FieldConstants.FIELD_Y_DIMENSION;
                SmartDashboard.putNumber("speed", speed);

                // Adds target pose
                m_field.getObject("target pose").setPose(new Pose2d(target_x, target_y,
                                new Rotation2d(swerve_position.getRotation().getRadians())));

                // Calculates difference between target and current x and y
                double x_diff = target_x - swerve_position.getX();
                double y_diff = target_y - swerve_position.getY();

                // Calculates magnitude
                double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
                double CLAMP_DISTANCE = 0.5;
                double clamped_magnitude = Math.min(1, magnitude / CLAMP_DISTANCE);

                // Calculates x and y speeds from magnitude and diff
                double SPEED_SCALING_FACTOR = 0.3;
                double x_speed = x_diff / magnitude * speed * SPEED_SCALING_FACTOR * clamped_magnitude;
                double y_speed = y_diff / magnitude * speed * SPEED_SCALING_FACTOR * clamped_magnitude;

                // Gets information needed to drive
                xSpeed = x_speed; // x_speed,
                ySpeed = y_speed; // y_speed,
                rot = spin;
                fieldRelative = true;

                // Returns
        }

        /**
         * Standard drive
         * runs calculations for auto-orient
         * 
         * @return
         *         DriveCommandData object with drive directions
         */
        public void calculate(DoubleControl _controls, double spin, Pose2d swerve_position, boolean field_relative) {

                // Gets speed level from controller
                double speedLevel = _controls.getSpeedLevel();

                // Gets information needed to drive
                xSpeed = _controls.getX() * speedLevel;
                ySpeed = _controls.getY() * speedLevel;
                rot = spin;
                fieldRelative = field_relative;
        }
}