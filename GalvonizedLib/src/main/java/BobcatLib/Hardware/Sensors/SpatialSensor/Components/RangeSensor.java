package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;

/**
 * The RangeSensor interface defines the basic operations for any range sensor, including methods
 * for retrieving the sensor's range, configuring the sensor, and retrieving the sensor's mode.
 * Implementations of this interface should provide specific logic for hardware interaction.
 */
public interface RangeSensor {

  /**
   * Gets the range (distance) in front of the sensor. This method should be implemented to return
   * the actual distance measurement from the sensor in millimeters.
   *
   * @return The range in millimeters.
   */
  public default double getRange() {
    return 0;
  }

  /**
   * Configures the sensor with default settings. This method should be implemented to apply the
   * default configuration for the sensor hardware, ensuring it's ready for operation.
   */
  public default void configRangeSensor() {}

  /**
   * Configures the sensor to the correct detection parameters based on the specified mode. The
   * implementation should update the sensor's settings to match the specified detection mode (e.g.,
   * short, medium, long range).
   *
   * @param mode The mode to configure the sensor with.
   */
  public default void configRangeSensor(DistanceMode mode) {}

  /**
   * Gets the current ranging mode of the sensor. This method should return the current mode of the
   * sensor, such as short, medium, or long range.
   *
   * @return The current ranging mode of the sensor.
   */
  public default DistanceMode getMode() {
    return null;
  }

  /**
   * Retrieves the optimal ranging mode based on the current sensor reading. This method should
   * calculate and return the best mode to use for the current range.
   *
   * @return The optimal mode for the sensor.
   */
  public default DistanceMode getOptimalMode() {
    return null;
  }
}
