package BobcatLib.Hardware.Encoders;

import BobcatLib.Hardware.Encoders.EncoderIO.EncoderIOInputs;

/**
 * A base class for interacting with an encoder using the {@link EncoderIO} interface. This class
 * provides methods for configuring the encoder, retrieving its position, and managing faults.
 */
public class BaseEncoder {
  /** The encoder IO interface for hardware communication. */
  private final EncoderIO io;

  /** Input data structure for encoder updates. */
  private final EncoderIOInputs inputs = new EncoderIOInputs();

  /**
   * Constructs a new {@code BaseEncoder} instance with the specified encoder IO interface.
   *
   * @param io the {@link EncoderIO} interface used to communicate with the encoder.
   */
  public BaseEncoder(EncoderIO io) {
    this.io = io;
  }

  /**
   * Periodically updates the encoder's inputs. This method should be called regularly to ensure
   * accurate data.
   */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Configures the absolute encoder's sensor position and determines its inversion state. This
   * method ensures the encoder is properly initialized for absolute position tracking.
   */
  public void configAbsEncoder() {
    io.configAbsEncoder();
    checkForFaults();
  }

  /**
   * Retrieves the absolute position of the encoder.
   *
   * @return the absolute position in rotations, represented as a value in the range (-1 to 0) or (0
   *     to 1).
   */
  public double getAbsolutePosition() {
    double absPos = io.getAbsolutePosition();
    checkForFaults();
    return absPos;
  }

  /**
   * Resets the encoder to its factory default settings.
   *
   * <p>Note: This method does not clear the stored zero offset of the encoder.
   */
  public void factoryDefault() {
    io.factoryDefault();
    checkForFaults();
  }

  /**
   * Clears any sticky faults on the encoder. Sticky faults are errors that persist until explicitly
   * cleared.
   */
  public void clearStickyFaults() {
    io.clearStickyFaults();
    checkForFaults();
  }

  /**
   * Checks the encoder for faults and handles them as necessary. This method is called internally
   * after critical operations.
   */
  public void checkForFaults() {
    io.checkForFaults();
  }
}
