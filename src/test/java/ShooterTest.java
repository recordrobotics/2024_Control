import frc.robot.Constants;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.Test;
import frc.robot.subsystems.Shooter;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import static org.junit.jupiter.api.Assertions.*;



public class ShooterTest {
    private Shooter shooter;
    private static final double TOLERANCE = 0.01;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        shooter = new Shooter();
    }

    @AfterEach
    public void shutdown() throws Exception {
        shooter.close();
        ShuffleboardUI.restartShuffleboard();
    }

    private void assertSpeed(double speed, String msg) {
        assertSpeed(speed, speed, msg);
    }

    private void assertSpeed(double speedL, double speedR, String msg) {
        assertEquals(-speedL, shooter.flywheelL.get(), TOLERANCE, msg);
        assertEquals(speedR, shooter.flywheelR.get(), TOLERANCE, msg);
    }

    @Test
    public void testInitialState() {
        assertSpeed(0, "Motor should be off initially.");
    }

    @Test
    public void testToggleSpeakerState() {
        shooter.toggle(Shooter.ShooterStates.SPEAKER);
        assertSpeed(Constants.Shooter.SPEAKER_SPEED, "Motor should be set to default SPEAKER speed.");
    }

    @Test
    public void testToggleAmpState() {
        shooter.toggle(Shooter.ShooterStates.AMP);
        assertSpeed(Constants.Shooter.AMP_SPEED, "Motor should be set to default AMP speed.");
    }

    @Test
    public void testToggleReverseState() {
        shooter.toggle(Shooter.ShooterStates.REVERSE);
        assertSpeed(Constants.Shooter.REVERSE_SPEED, "Motor should be set to default REVERSE speed.");
    }

    @Test
    public void testToggleCustomState() {
        shooter.toggle(1.5);
        assertSpeed(1, "Motor should not go above 1 speed");
    }

    @Test
    public void testToggleNegativeCustomState() {
        shooter.toggle(-1.5);
        assertSpeed(-1, "Motor should not go below -1 speed");
    }

    public void testLeftPositiveRightNegative() {
        shooter.toggle(1, -1);
        assertSpeed(1, -1, "Left forward right backward");
    }

    @Test
    public void testToggleOffState() {
        shooter.toggle(Shooter.ShooterStates.SPEAKER); // Set to IN first
        shooter.toggle(Shooter.ShooterStates.OFF); // Now set to OFF
        assertSpeed(0, "Motor should be off when state is OFF.");
    }

    @Test
    public void testKill() {
        shooter.toggle(Shooter.ShooterStates.SPEAKER); // Activate the motor
        shooter.kill(); // Kill should turn off the motor
        assertSpeed(0, "Motor should be off after kill.");
    }
}