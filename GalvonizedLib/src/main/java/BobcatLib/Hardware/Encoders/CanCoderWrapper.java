package BobcatLib.Hardware.Encoders;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.FaultsAndErrors.CanCoderFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

/**
 * A wrapper class for managing a CANcoder device, implementing the {@link EncoderIO} interface.
 * This class handles configuration, fault checking, and interaction with the encoder.
 */
public class CanCoderWrapper implements EncoderIO {
  /** Configuration for the CANcoder device. */
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  /** The CANcoder hardware instance. */
  private CANcoder encoder;

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

  /**
   * Constructs a {@code CanCoderWrapper} instance.
   *
   * @param details Details of the CAN device, including its ID and name.
   * @param chosenModule The encoder constants for the chosen module.
   * @param canivorename The CANivore name to use for the device.
   */
  public CanCoderWrapper(
      CANDeviceDetails details, EncoderConstants chosenModule, String canivorename) {
    this.details = details;
    int id = details.getDeviceNumber();
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    configAbsEncoder();

    /* Angle Encoder Config */
    if (canivorename == "" || canivorename == "rio") {
      encoder = new CANcoder(id);
    } else {
      encoder = new CANcoder(id, canivorename);
    }
    encoder.getConfigurator().apply(swerveCANcoderConfig);

    faults = new CanCoderFaults(encoder, id);
  }

  /**
   * Updates the Encoder inputs.
   *
   * @param inputs The EncoderIOInputs object to populate with motor data.
   */
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.getEncoderPosition = Rotation2d.fromRotations(getAbsolutePosition());
    inputs.faulted = faults.hasFaultOccured();
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/IsProLicensed",
        encoder.getIsProLicensed().getValue());
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/RelativePosition",
        getRelativePosition());
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/AbsolutePosition",
        inputs.getEncoderPosition);
  }

  /**
   * Configures the absolute encoder's sensor position and determines its inversion state. This
   * method initializes the encoder based on the chosen module's configuration.
   */
  public void configAbsEncoder() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection =
        chosenModule.absoluteEncoderInvert.asSensorDirectionValue();
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Retrieves the absolute position of the encoder.
   *
   * @return The absolute position of the encoder, as a rotation value in the range (-0.5 to
   *     0.999999).
   */
  public double getAbsolutePosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Retrieves the relative position of the encoder.
   *
   * @return The relative position of the encoder, as a rotation value Minimum Value: -16384.0
   *     Maximum Value: 16383.999755859375
   */
  public Rotation2d getRelativePosition() {
    Rotation2d relPosition = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
    return relPosition;
  }

  /** Resets the encoder to its factory default configuration. */
  public void factoryDefault() {
    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  /**
   * Clears sticky faults on the encoder. Sticky faults are persistent errors that must be manually
   * cleared.
   */
  public void clearStickyFaults() {
    encoder.clearStickyFaults();
  }

  /**
   * Checks for faults in the encoder. This method delegates fault checking to the {@link
   * CanCoderFaults} instance.
   */
  public void checkForFaults() {
    boolean isFaulted = faults.hasFaultOccured();
  }
}
