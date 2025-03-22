package BobcatLib.Hardware.Gyros.Parser;

/**
 * Represents the configuration of a gyroscopic device in JSON format.
 *
 * <p>This class is immutable and is designed to hold the properties of a gyroscopic device, such as
 * its type, ID, CAN bus name, and inversion state.
 *
 * <p>Instances of this class can be created using the provided constructors.
 *
 * @since 1.0
 */
public final class GyroDeviceJson {

  /** The device type, e.g., pigeon, pigeon2, sparkmax, talonfx, navx. */
  private final String type;

  /** The CAN ID or pin ID of the device. */
  private final int id;

  /** The CAN bus name on which the device resides, if using CAN. */
  private final String canbus;

  /** Indicates whether the gyro is inverted. */
  private final boolean inverted;

  /**
   * Constructs a new {@code GyroDeviceJson} with default values.
   *
   * <p>The default values are:
   *
   * <ul>
   *   <li>Type: empty string
   *   <li>ID: 0
   *   <li>CAN bus: empty string
   *   <li>Inverted: false
   * </ul>
   */
  public GyroDeviceJson() {
    this("", 0, "", false);
  }

  /**
   * Constructs a new {@code GyroDeviceJson} with the specified ID and CAN bus name.
   *
   * @param id The CAN ID or pin ID of the device.
   * @param canbus The CAN bus name where the device resides, if applicable.
   */
  public GyroDeviceJson(int id, String canbus) {
    this("", id, canbus, false);
  }

  /**
   * Constructs a new {@code GyroDeviceJson} with all specified properties.
   *
   * @param type The type of the device.
   * @param id The CAN ID or pin ID of the device.
   * @param canbus The CAN bus name where the device resides, if applicable.
   * @param inverted The inversion state of the gyro.
   */
  public GyroDeviceJson(String type, int id, String canbus, boolean inverted) {
    this.type = type;
    this.id = id;
    this.canbus = canbus;
    this.inverted = inverted;
  }

  /**
   * Gets the type of the device.
   *
   * @return The device type, e.g., pigeon, pigeon2, sparkmax, talonfx, navx.
   */
  public String getType() {
    return type;
  }

  /**
   * Gets the CAN ID or pin ID of the device.
   *
   * @return The CAN ID or pin ID.
   */
  public int getId() {
    return id;
  }

  /**
   * Gets the CAN bus name where the device resides.
   *
   * @return The CAN bus name.
   */
  public String getCanbus() {
    return canbus;
  }

  /**
   * Gets the inversion state of the gyro.
   *
   * @return {@code true} if the gyro is inverted; {@code false} otherwise.
   */
  public boolean isInverted() {
    return inverted;
  }
}
