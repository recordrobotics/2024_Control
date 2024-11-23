package frc.robot.utils.simulation;

import edu.wpi.first.math.system.plant.DCMotor;

public class CANSparkMaxSim {
  @SuppressWarnings("unused") // TODO
  private DCMotor motor; // TODO: simulate speeds with edu.wpi.first.wpilibj.simulation.FlywheelSim

  private double currentSpeed;
  private static final double minSpeed = -1;
  private static final double maxSpeed = 1;

  public CANSparkMaxSim() {
    // motor = DCMotor.getNEO(1); // TODO
  }

  private double clamp(double old) {
    if (old < minSpeed) {
      return minSpeed;
    } else if (old > maxSpeed) {
      return maxSpeed;
    } else {
      return old;
    }
  }

  public void set(double speed) {
    currentSpeed = clamp(speed);
  }

  public double get() {
    return currentSpeed;
  }
}

/* its your job to add flywheelsim
 * be warned, the docs are terrible
 * its very confusing
 * dont try at home
 * will cause banging head against wall
 * speaking from expirence here
 * TODO
 * TODO
 * TODO
 * TODO
 * TODO
 * TODO
 * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/FlywheelSim.html
 */
