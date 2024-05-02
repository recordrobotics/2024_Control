package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;


public class SwerveDriveSim{

  // use differential to program

    private final DCMotor m_motor;
    private final double m_originalGearing;
    private final Matrix<N7, N1> m_measurementStdDevs;
    private double m_currentGearing;
    private final double m_wheelRadiusMeters;

    private Matrix<N2, N1> m_u;
    private Matrix<N7, N1> m_x;
    private Matrix<N7, N1> m_y;

    private final double m_rb;
    private final LinearSystem<N2, N2, N2> m_plant;

    public SwerveDriveSim(LinearSystem<N2, N2, N2> plant,
    DCMotor driveMotor,
    double gearing,
    double swerveWidthMeters,
    double wheelRadiusMeters,
    Matrix<N7, N1> measurementStdDevs) {
      this.m_plant = plant;
      this.m_rb = swerveWidthMeters / 2.0;
      this.m_motor = driveMotor;
      this.m_originalGearing = gearing;
      this.m_measurementStdDevs = measurementStdDevs;
      m_wheelRadiusMeters = wheelRadiusMeters;
      m_currentGearing = m_originalGearing;

      m_x = new Matrix<>(Nat.N7(), Nat.N1());
      m_u = VecBuilder.fill(0, 0);
      m_y = new Matrix<>(Nat.N7(), Nat.N1());
    }

     /**
   * Sets the applied voltage to the drivetrain. Note that positive voltage must make that side of
   * the drivetrain travel forward (+X).
   *
   * @param leftVoltageVolts The left voltage.
   * @param rightVoltageVolts The right voltage.
   */
  public void setInputs(double leftVoltageVolts, double rightVoltageVolts) {
    m_u = clampInput(VecBuilder.fill(leftVoltageVolts, rightVoltageVolts));
  }

  /**
   * Update the drivetrain states with the current time difference.
   *
   * @param dtSeconds the time difference
   */
  public void update(double dtSeconds) {
    m_x = NumericalIntegration.rkdp(this::getDynamics, m_x, m_u, dtSeconds);
    m_y = m_x;
    if (m_measurementStdDevs != null) {
      m_y = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
    }
  }

  /** Returns the full simulated state of the drivetrain. */
  Matrix<N7, N1> getState() {
    return m_x;
  }

  /**
   * Get one of the drivetrain states.
   *
   * @param state the state to get
   * @return the state
   */
  double getState(State state) {
    return m_x.get(state.value, 0);
  }

  private double getOutput(State output) {
    return m_y.get(output.value, 0);
  }

  /**
   * Returns the direction the robot is pointing.
   *
   * <p>Note that this angle is counterclockwise-positive, while most gyros are clockwise positive.
   *
   * @return The direction the robot is pointing.
   */
  public Rotation2d getHeading() {
    return new Rotation2d(getOutput(State.kHeading));
  }

  /**
   * Returns the current pose.
   *
   * @return The current pose.
   */
  public Pose2d getPose() {
    return new Pose2d(getOutput(State.kX), getOutput(State.kY), getHeading());
  }

  /**
   * Get the right encoder position in meters.
   *
   * @return The encoder position.
   */
  public double getRightPositionMeters() {
    return getOutput(State.kRightPosition);
  }

  /**
   * Get the right encoder velocity in meters per second.
   *
   * @return The encoder velocity.
   */
  public double getRightVelocityMetersPerSecond() {
    return getOutput(State.kRightVelocity);
  }

  /**
   * Get the left encoder position in meters.
   *
   * @return The encoder position.
   */
  public double getLeftPositionMeters() {
    return getOutput(State.kLeftPosition);
  }

  /**
   * Get the left encoder velocity in meters per second.
   *
   * @return The encoder velocity.
   */
  public double getLeftVelocityMetersPerSecond() {
    return getOutput(State.kLeftVelocity);
  }

  /**
   * Get the current draw of the left side of the drivetrain.
   *
   * @return the drivetrain's left side current draw, in amps
   */
  public double getLeftCurrentDrawAmps() {
    return m_motor.getCurrent(
            getState(State.kLeftVelocity) * m_currentGearing / m_wheelRadiusMeters, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Get the current draw of the right side of the drivetrain.
   *
   * @return the drivetrain's right side current draw, in amps
   */
  public double getRightCurrentDrawAmps() {
    return m_motor.getCurrent(
            getState(State.kRightVelocity) * m_currentGearing / m_wheelRadiusMeters, m_u.get(1, 0))
        * Math.signum(m_u.get(1, 0));
  }

  /**
   * Get the current draw of the drivetrain.
   *
   * @return the current draw, in amps
   */
  public double getCurrentDrawAmps() {
    return getLeftCurrentDrawAmps() + getRightCurrentDrawAmps();
  }

  /**
   * Get the drivetrain gearing.
   *
   * @return the gearing ration
   */
  public double getCurrentGearing() {
    return m_currentGearing;
  }

  /**
   * Sets the gearing reduction on the drivetrain. This is commonly used for shifting drivetrains.
   *
   * @param newGearRatio The new gear ratio, as output over input.
   */
  public void setCurrentGearing(double newGearRatio) {
    this.m_currentGearing = newGearRatio;
  }

  /**
   * Sets the system state.
   *
   * @param state The state.
   */
  public void setState(Matrix<N7, N1> state) {
    m_x = state;
  }

  /**
   * Sets the system pose.
   *
   * @param pose The pose.
   */
  public void setPose(Pose2d pose) {
    m_x.set(State.kX.value, 0, pose.getX());
    m_x.set(State.kY.value, 0, pose.getY());
    m_x.set(State.kHeading.value, 0, pose.getRotation().getRadians());
    m_x.set(State.kLeftPosition.value, 0, 0);
    m_x.set(State.kRightPosition.value, 0, 0);
  }

  protected Matrix<N7, N1> getDynamics(Matrix<N7, N1> x, Matrix<N2, N1> u) {

  }

  protected Matrix<N2, N1> clampInput(Matrix<N2, N1> u) {
    return StateSpaceUtil.desaturateInputVector(u, RobotController.getBatteryVoltage());
  }

  enum State {

  }

}
