package BobcatLib.Hardware.Motors.SensorHelpers;

import com.ctre.phoenix6.signals.InvertedValue;

/**
 * Utility wrapper to Invert state of the device. Incredibily usefull for modularization of the
 * different device types.
 */
public class InvertedWrapper {
  /** is inverted internal state. */
  public boolean isInverted = false;
  /**
   * Constructor
   *
   * @param mode input as boolean
   */
  public InvertedWrapper(boolean mode) {
    isInverted = mode;
  }
  /**
   * Concstructor
   *
   * @param mode input mode as inverted value
   */
  public InvertedWrapper(InvertedValue mode) {
    isInverted = false;
    if (mode == InvertedValue.Clockwise_Positive) {
      isInverted = true;
    }
  }
  /**
   * Returns Inverted Value as CTRE equivalent
   *
   * @return InvertedValue
   */
  public InvertedValue asCTRE() {
    if (isInverted) {
      return InvertedValue.Clockwise_Positive;
    }
    return InvertedValue.CounterClockwise_Positive;
  }
  /**
   * Returns Inverted Wrapper as Rev Equivalent
   *
   * @return boolean
   */
  public boolean asREV() {
    return isInverted;
  }
  /**
   * Returns Inverted Wrapper as Thrifty Equivalent
   *
   * @return boolean
   */
  public boolean asThrifty() {
    return isInverted;
  }
}
