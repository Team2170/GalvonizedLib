package BobcatLib.Hardware.Motors.SensorHelpers;

import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * A wrapper class for managing the direction of a sensor. This class handles the inversion of
 * sensor direction and allows conversion between boolean and `SensorDirectionValue`
 * representations.
 */
public class SensorDirectionWrapper {
  /** Internal state representing whether the sensor is inverted. */
  public boolean isInverted = false;

  /**
   * Constructor to initialize the sensor direction wrapper with a boolean mode.
   *
   * @param mode A boolean value representing the inversion state of the sensor. `true` means the
   *     sensor is inverted, `false` means it is not.
   */
  public SensorDirectionWrapper(boolean mode) {
    isInverted = mode;
  }

  /**
   * Constructor to initialize the sensor direction wrapper with a `SensorDirectionValue` mode.
   *
   * @param mode A `SensorDirectionValue` representing the sensor direction. If the mode is
   *     `Clockwise_Positive`, the sensor will be considered inverted.
   */
  public SensorDirectionWrapper(SensorDirectionValue mode) {
    isInverted = false;
    if (mode == SensorDirectionValue.Clockwise_Positive) {
      isInverted = true;
    }
  }

  /**
   * Returns the sensor direction as a `SensorDirectionValue`.
   *
   * @return A `SensorDirectionValue` that represents the sensor direction. Returns
   *     `Clockwise_Positive` if inverted, `CounterClockwise_Positive` otherwise.
   */
  public SensorDirectionValue asSensorDirectionValue() {
    if (isInverted) {
      return SensorDirectionValue.Clockwise_Positive;
    }
    return SensorDirectionValue.CounterClockwise_Positive;
  }

  /**
   * Returns the sensor direction as a boolean value.
   *
   * @return `true` if the sensor is inverted, `false` otherwise.
   */
  public boolean asBoolean() {
    return isInverted;
  }
}
