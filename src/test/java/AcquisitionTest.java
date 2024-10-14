import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Acquisition;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;

public class AcquisitionTest {
    private Acquisition acquisition;
    private PWMSim motorSim;
    private static final double TOLERANCE = 0.01;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        acquisition = new Acquisition(true);
        motorSim = new PWMSim(RobotMap.Acquisition.ACQUISITION_MOTOR_ID); // Simulate the motor with PWMSim
    }

    @AfterEach
    public void shutdown() throws Exception {
        acquisition.close();
    }

    @Test
    public void testInitialState() {
        assertEquals(Acquisition.AcquisitionStates.OFF, acquisition.acquisitionState, "Initial state should be OFF.");
        assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off initially.");
    }

    @Test
    public void testToggleInState() {
        acquisition.toggle(Acquisition.AcquisitionStates.IN);
        assertEquals(Acquisition.AcquisitionStates.IN, acquisition.acquisitionState, "State should be IN.");
        assertEquals(Constants.Acquisition.ACQUISITION_SPEED, motorSim.getSpeed(), TOLERANCE, "Motor should be set to default IN speed.");
    }

    @Test
    public void testToggleReverseState() {
        acquisition.toggle(Acquisition.AcquisitionStates.REVERSE);
        assertEquals(Acquisition.AcquisitionStates.REVERSE, acquisition.acquisitionState, "State should be REVERSE.");
        assertEquals(-Constants.Acquisition.ACQUISITION_SPEED, motorSim.getSpeed(), TOLERANCE, "Motor should be set to default REVERSE speed.");
    }

    @Test
    public void testToggleOffState() {
        acquisition.toggle(Acquisition.AcquisitionStates.IN); // Set to IN first
        acquisition.toggle(Acquisition.AcquisitionStates.OFF); // Now set to OFF
        assertEquals(Acquisition.AcquisitionStates.OFF, acquisition.acquisitionState, "State should be OFF.");
        assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off when state is OFF.");
    }

    @Test
    public void testKill() {
        acquisition.toggle(Acquisition.AcquisitionStates.IN); // Activate the motor
        acquisition.kill(); // Kill should turn off the motor
        assertEquals(Acquisition.AcquisitionStates.OFF, acquisition.acquisitionState, "Kill should set state to OFF.");
        assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off after kill.");
    }
}