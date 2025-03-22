package BobcatLib.Hardware.Encoders;

import BobcatLib.Utilities.CANDeviceDetails;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The ThriftyAbsoluteEncoder class implements the EncoderIO interface and provides methods for
 * working with an absolute encoder connected via analog input. It supports configuring the
 * encoder's inversion, getting its absolute position, and managing faults (though no actual faults
 * management is implemented for this encoder type).
 */
public class ThriftyAbsoluteEncoder implements EncoderIO {

  /** The analog input object used to read the encoder's voltage. */
  public AnalogInput encoder;

  /** Indicates whether the encoder is inverted. */
  private boolean isInverted;

  /** The encoder configuration constants specific to this module. */
  public EncoderConstants chosenModule;

  /** Details about the CAN device associated with the encoder. */
  private CANDeviceDetails details;

  /**
   * Constructor for creating a ThriftyAbsoluteEncoder instance. This method sets up the encoder
   * based on the provided device details and configuration.
   *
   * @param details The CAN device details (e.g., CAN ID).
   * @param chosenModule The encoder configuration constants (e.g., inversion settings).
   */
  public ThriftyAbsoluteEncoder(CANDeviceDetails details, EncoderConstants chosenModule) {
    this.details = details;
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = new AnalogInput(details.getDeviceNumber());
  }

  /**
   * Configures the absolute encoder sensor, including setting its inversion. This method uses the
   * configuration settings to apply the correct inversion logic.
   */
  public void configAbsEncoder() {
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Gets the absolute position of the encoder. The position is returned as a normalized rotation
   * value between -1.0 and 1.0, with the sign of the value adjusted based on the encoder's
   * inversion setting.
   *
   * @return The absolute position of the encoder in the range (-1 to 1).
   */
  public double getAbsolutePosition() {
    return (isInverted ? -1.0 : 1.0)
        * (encoder.getAverageVoltage() / RobotController.getVoltage5V());
  }

  /**
   * Resets the encoder to its factory defaults. For this encoder type, this method does nothing as
   * there are no configurable settings to reset.
   */
  public void factoryDefault() {
    // No reset required for this encoder type.
  }

  /**
   * Clears sticky faults on the encoder. For this encoder type, this method does nothing as no
   * sticky faults are managed.
   */
  public void clearStickyFaults() {
    // No fault clearing required for this encoder type.
  }

  /**
   * Checks for any faults in the encoder. For this encoder type, this method does nothing as no
   * faults are managed.
   */
  public void checkForFaults() {
    // No fault checking required for this encoder type.
  }
}
