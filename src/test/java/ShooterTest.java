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
        this.shooter = new Shooter();
    }

    @AfterEach
    public void shutdown() throws Exception {
        this.shooter.close();
    }

    private void assertSpeed(double speed, String msg) {
        assertSpeed(speed, speed, msg);
    }

    private void assertSpeed(double speedL, double speedR, String msg) {
        assertEquals(-speedL, this.shooter.flywheelL.get(), TOLERANCE, msg);
        assertEquals(speedR, this.shooter.flywheelR.get(), TOLERANCE, msg);
    }

    @Test
    public void testInitialState() {
        assertSpeed(0, "Motor should be off initially.");
    }

    @Test
    public void testToggleSpeakerState() {
        this.shooter.toggle(Shooter.ShooterStates.SPEAKER);
        assertSpeed(Constants.Shooter.SPEAKER_SPEED, "Motor should be set to default SPEAKER speed.");
    }

    @Test
    public void testToggleAmpState() {
        this.shooter.toggle(Shooter.ShooterStates.AMP);
        assertSpeed(Constants.Shooter.AMP_SPEED, "Motor should be set to default AMP speed.");
    }

    @Test
    public void testToggleReverseState() {
        this.shooter.toggle(Shooter.ShooterStates.REVERSE);
        assertSpeed(Constants.Shooter.REVERSE_SPEED, "Motor should be set to default REVERSE speed.");
    }

    @Test
    public void testToggleCustomState() {
        this.shooter.toggle(1.5);
        assertSpeed(1, "Motor should not go above 1 speed");
    }

    @Test
    public void testToggleNegativeCustomState() {
        this.shooter.toggle(-1.5);
        assertSpeed(-1, "Motor should not go below -1 speed");
    }

    @Test
    public void testLeftPositiveRightNegative() {
        this.shooter.toggle(1, -1);
        assertSpeed(1, -1, "Left forward right backward");
    }

    @Test
    public void testToggleOffState() {
        this.shooter.toggle(Shooter.ShooterStates.SPEAKER); // Set to IN first
        this.shooter.toggle(Shooter.ShooterStates.OFF); // Now set to OFF
        assertSpeed(0, "Motor should be off when state is OFF.");
    }

    @Test
    public void testKill() {
        this.shooter.toggle(Shooter.ShooterStates.SPEAKER); // Activate the motor
        this.shooter.kill(); // Kill should turn off the motor
        assertSpeed(0, "Motor should be off after kill.");
    }
}