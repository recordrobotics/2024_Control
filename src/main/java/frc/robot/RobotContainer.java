package frc.robot;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.*;
import frc.robot.commands.manual.*;
import frc.robot.commands.notes.*;
import frc.robot.control.*;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands are defined here
  private final Drivetrain _drivetrain;
  private final Shooter _shooter;
  private final Crashbar _crashbar;
  private final Climbers _climbers;
  private final Acquisition _acquisition;
  private final Channel _channel;
  private final Photosensor _photosensor;
  private final PCMCompressor _compressor;
  private final Limelight _limelight;

  // Autonomous
  private final AutoPath _autoPath;
  private Command autoCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init subsystems
    _drivetrain = new Drivetrain();
    _channel = new Channel();
    _acquisition = new Acquisition();
    _shooter = new Shooter();
    _crashbar = new Crashbar();
    _photosensor = new Photosensor();
    _climbers = new Climbers();
    _compressor = new PCMCompressor();
    _limelight = new Limelight(_drivetrain);

    // Sets up auto path
    _autoPath = new AutoPath(_drivetrain, _acquisition, _photosensor, _channel, _shooter, _crashbar);

    // Sets up Control scheme chooser
    ShuffleboardUI.Overview.addControls(
      new JoystickXbox(2, 0),
      new DoubleXbox(0, 1),
      new DoubleXboxSpin(0, 1)
    );

    // Bindings and Teleop
    configureButtonBindings();

    // Setup other dashboards
    setup_test_dashboard();
  }

  private void setup_test_dashboard() {
    // TODO move the rest of dashboard setup here and out of substem classes.
    // This makes sure we don't accidentally pull in the dashboards while running tests.

    ShuffleboardUI.Test
        .addSlider("Flywheel Left", _shooter.flywheelL.get(), -1, 1) // LEFT set slider to show value between -1 and 1
        .subscribe(_shooter.flywheelL::set); // LEFT if the slider is moved, call flywheelL.set
    ShuffleboardUI.Test
        .addSlider("Flywheel Right", _shooter.flywheelR.get(), -1, 1) // RIGHT set slider to show value between -1 and 1
        .subscribe(_shooter.flywheelR::set); // RIGHT if the slider is moved, call flywheelR.set
  }

  public void teleopInit() {
    // Sets default command for manual swerve. It is the only one right now
    _drivetrain.setDefaultCommand(new ManualSwerve(_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getKillAuto()).whileTrue(new KillSpecified(_drivetrain, _acquisition, _channel, _shooter, _crashbar, _climbers));

    // Command to kill compressor
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getKillCompressor())
                .onTrue(new InstantCommand(_compressor::disable))
                .onFalse(new InstantCommand(_compressor::enable))
                .onTrue(new InstantCommand(()->ShuffleboardUI.Overview.getControl().vibrate(1)))
                .onFalse(new InstantCommand(()->ShuffleboardUI.Overview.getControl().vibrate(0)));

    // Notes triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getAcquire()).toggleOnTrue(new AcquireSmart(_acquisition, _channel, _photosensor, _shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootSpeaker()).toggleOnTrue(new ShootSpeaker(_channel, _shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootAmp()).toggleOnTrue(new ShootAmp(_channel, _shooter, _crashbar));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getReverse()).whileTrue(new ManualReverse(_acquisition, _channel));

    // Manual triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootAmp()).toggleOnTrue(new ManualShooter(_shooter, Shooter.ShooterStates.AMP));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootSpeaker()).toggleOnTrue(new ManualShooter(_shooter, Shooter.ShooterStates.SPEAKER));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualCrashbar()).toggleOnTrue(new ManualCrashbar(_crashbar));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualAcquisition()).whileTrue(new ManualAcquisition(_acquisition, _channel));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualClimbers()).toggleOnTrue(new ManualClimbers(_climbers));

    // Reset pose trigger
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getPoseReset()).onTrue(new InstantCommand(_drivetrain::resetDriverPose));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(_drivetrain, _autoPath);
    }
    return autoCommand;
  }

  public void testPeriodic() {
    ShuffleboardUI.Test.testPeriodic();
  }

  /**
   * frees up all hardware allocations
   */
  public void close() {
      _drivetrain.close();
      _channel.close();
      _acquisition.close();
      _shooter.close();
      _crashbar.close();
      _photosensor.close();
      _climbers.close();
      _compressor.close();
      _limelight.close();
  }
}