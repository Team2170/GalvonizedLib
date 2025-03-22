package BobcatLib.Hardware.Encoders;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.FaultsAndErrors.CanCoderFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimDouble;

public class EncoderSim implements EncoderIO {
  /** Indicates whether the last angle reading was faulty. */
  public boolean readingError = false;

  /** Indicates whether the encoder's direction is inverted. */
  public boolean isInverted = false;

  /**
   * An {@link Alert} for notifying if the CAN ID is greater than 40, which can cause undefined
   * behavior.
   */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  /** The encoder constants for the chosen module. */
  public EncoderConstants chosenModule;

  /** Fault handler for the CANcoder device. */
  private CanCoderFaults faults;

  /** Details of the CAN device. */
  private CANDeviceDetails details;

  public SimDevice simDevice;
  public SimDouble simAngle;

  /**
   * Constructs a {@code CanCoderWrapper} instance.
   *
   * @param details Details of the CAN device, including its ID and name.
   * @param chosenModule The encoder constants for the chosen module.
   * @param canivorename The CANivore name to use for the device.
   */
  public EncoderSim(CANDeviceDetails details, EncoderConstants chosenModule, String canivorename) {
    this.details = details;
    int id = details.getDeviceNumber();
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    configAbsEncoder();

    simDevice = new SimDevice(SimDeviceJNI.createSimDevice("SimEncoder [" + id + "]"));
    simAngle = simDevice.createDouble("angle", Direction.kBidir, 0.00);
  }

  /**
   * Configures the absolute encoder's sensor position and determines its inversion state. This
   * method initializes the encoder based on the chosen module's configuration.
   */
  public void configAbsEncoder() {
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Retrieves the absolute position of the encoder.
   *
   * @return The absolute position of the encoder, as a rotation value in the range (-0.5 to
   *     0.999999).
   */
  public double getAbsolutePosition() {
    return simAngle.get();
  }

  /** Resets the encoder to its factory default configuration. */
  public void factoryDefault() {
    simAngle.set(0);
  }

  /**
   * Clears sticky faults on the encoder. Sticky faults are persistent errors that must be manually
   * cleared.
   */
  public void clearStickyFaults() {}

  /**
   * Checks for faults in the encoder. This method delegates fault checking to the {@link
   * CanCoderFaults} instance.
   */
  public void checkForFaults() {}
}
