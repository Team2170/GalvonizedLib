package GalvonizedLib.Utilities;

/**
 * This class represents the details of a CAN (Controller Area Network) device. It includes
 * information such as the device's manufacturer, device number, bus, and subsystem name. This class
 * is useful for managing and identifying CAN devices in a robotic system.
 */
public class CANDeviceDetails {

  /** Enum representing the manufacturer of the CAN device. */
  public enum Manufacturer {
    Unknown, // Unknown vendor
    Thrifty, // Thrifty vendor
    Grapple, // Grapple vendor
    Pwf, // Pwf vendor
    Redux, // Redux vendor
    Rev, // Rev vendor
    Ctre // Ctre vendor
  }

  private final Manufacturer manufacturer;
  private final int deviceNumber;
  private final String bus;
  private final String subsystemName;

  /**
   * Constructs a CANDeviceDetails object with the specified device number, bus name, manufacturer,
   * and subsystem name.
   *
   * @param deviceNumber The unique identifier for the CAN device.
   * @param bus The bus name to which the device is connected.
   * @param manufacturer The manufacturer of the CAN device.
   * @param subsystemName The name of the subsystem this device is associated with.
   */
  public CANDeviceDetails(
      int deviceNumber, String bus, Manufacturer manufacturer, String subsystemName) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
    this.manufacturer = manufacturer;
    this.subsystemName = subsystemName;
  }

  /**
   * Constructs a CANDeviceDetails object with the specified device number, bus name, and
   * manufacturer. The subsystem name will be set to an empty string.
   *
   * @param deviceNumber The unique identifier for the CAN device.
   * @param bus The bus name to which the device is connected.
   * @param manufacturer The manufacturer of the CAN device.
   */
  public CANDeviceDetails(int deviceNumber, String bus, Manufacturer manufacturer) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
    this.manufacturer = manufacturer;
    this.subsystemName = "";
  }

  /**
   * Constructs a CANDeviceDetails object with the specified device number and manufacturer. The bus
   * name will be set to an empty string, and the subsystem name will be set to an empty string.
   *
   * @param deviceNumber The unique identifier for the CAN device.
   * @param manufacturer The manufacturer of the CAN device.
   */
  public CANDeviceDetails(int deviceNumber, Manufacturer manufacturer) {
    this(deviceNumber, "", manufacturer);
  }

  /**
   * Constructs a CANDeviceDetails object with the specified device number, manufacturer, and
   * subsystem name. The bus name will be set to an empty string.
   *
   * @param deviceNumber The unique identifier for the CAN device.
   * @param manufacturer The manufacturer of the CAN device.
   * @param subsystemName The name of the subsystem this device is associated with.
   */
  public CANDeviceDetails(int deviceNumber, Manufacturer manufacturer, String subsystemName) {
    this(deviceNumber, "", manufacturer, subsystemName);
  }

  /**
   * Constructs a CANDeviceDetails object with the specified device number. The bus name will be set
   * to an empty string, and the manufacturer will be set to Unknown.
   *
   * @param deviceNumber The unique identifier for the CAN device.
   */
  public CANDeviceDetails(int deviceNumber) {
    this(deviceNumber, "", Manufacturer.Unknown);
  }

  /**
   * Gets the manufacturer type of the CAN device.
   *
   * @return The manufacturer of the CAN device.
   */
  public Manufacturer getManufacturer() {
    return manufacturer;
  }

  /**
   * Gets the device number of the CAN device.
   *
   * @return The device number of the CAN device.
   */
  public int getDeviceNumber() {
    return deviceNumber;
  }

  /**
   * Gets the bus name of the CAN device.
   *
   * @return The bus name of the CAN device.
   */
  public String getBus() {
    return bus;
  }

  /**
   * Gets the subsystem name this CAN device is associated with.
   *
   * @return The subsystem name of the CAN device.
   */
  public String getSubsysemName() {
    return subsystemName;
  }

  /**
   * Compares this CANDeviceDetails object with another to determine if they are equal.
   *
   * @param other The other CANDeviceDetails object to compare.
   * @return {@code true} if the device numbers, bus, manufacturer, and subsystem name are all the
   *     same; {@code false} otherwise.
   */
  public boolean equals(CANDeviceDetails other) {
    return other.deviceNumber == deviceNumber
        && other.bus.equals(bus)
        && other.manufacturer == manufacturer
        && other.subsystemName.equals(subsystemName);
  }
}
