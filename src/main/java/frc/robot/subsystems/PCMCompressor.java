package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PCMCompressor extends SubsystemBase {
    // Creates AHRS _nav object
    private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public static void initPCM() {
    }

    public void disable() {
        compressor.disable();
    }

    public void enable() {
        compressor.enableDigital();
    }

    public double getCurrent() {
        return compressor.getCurrent();
    }

    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    public boolean isPumping() {
        return compressor.getPressureSwitchValue();
    }

    // Navsensor Constructor, Create instance
    public PCMCompressor() {
    }

    @Override
    public void periodic() {
    }
}
