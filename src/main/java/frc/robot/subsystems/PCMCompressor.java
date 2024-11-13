package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PCMCompressor extends SubsystemBase {
  private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private static boolean isDisabledManually = false;

  public PCMCompressor() {
    ShuffleboardUI.Overview.setCompressor(this::isEnabled);
    ShuffleboardUI.Overview.setCompressorManuallyDisabled(this::isDisabledManually);
  }

  public void disable() {
    try {
      compressor.disable();
      isDisabledManually = true;
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
    }
  }

  public void enable() {
    try {
      compressor.enableDigital();
      isDisabledManually = false;
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
    }
  }

  public double getCurrent() {
    try {
      return compressor.getCurrent();
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
      return 0.0; // TODO should this be -1.0, 0.0, or NaN?
    }
  }

  public boolean isEnabled() {
    try {
      return compressor.isEnabled();
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
      return false;
    }
  }

  public boolean isDisabledManually() {
    return isDisabledManually;
  }

  public boolean isFull() {
    try {
      return compressor.getPressureSwitchValue();
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
      return false; // TODO what shoud this be?
    }
  }

  public boolean isPumping() {
    return this.isFull() && this.isEnabled() && !this.isDisabledManually();
  }

  @Override
  public void periodic() {}
}
