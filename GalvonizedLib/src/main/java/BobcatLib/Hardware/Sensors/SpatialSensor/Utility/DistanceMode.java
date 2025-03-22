package BobcatLib.Hardware.Sensors.SpatialSensor.Utility;

import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * The DistanceMode class represents the different distance modes used by various Time-of-Flight
 * (ToF) sensors. This class allows conversion between different ranging modes used in various
 * sensor libraries, such as LaserCAN and PWF.
 */
public class DistanceMode {

  /**
   * Enumeration of possible distance modes.
   *
   * <p>SHORT: Short range mode
   *
   * <p>MEDIUM: Medium range mode
   *
   * <p>LONG: Long range mode
   */
  public enum modes {
    SHORT,
    MEDIUM,
    LONG
  }

  public modes currentMode;

  /**
   * Constructs a new DistanceMode instance with the specified mode.
   *
   * @param cm The distance mode (e.g., SHORT, MEDIUM, LONG).
   */
  public DistanceMode(modes cm) {
    this.currentMode = cm;
  }

  /**
   * Default constructor for the DistanceMode class. Initializes the distance mode to null (or
   * undefined).
   */
  public DistanceMode() {}

  /**
   * Converts the current distance mode to a LaserCAN compatible ranging mode.
   *
   * @return The corresponding LaserCAN RangingMode (SHORT, MEDIUM, LONG), or null if no valid mode.
   */
  public au.grapplerobotics.interfaces.LaserCanInterface.RangingMode asLaserCAN() {
    switch (currentMode) {
      case SHORT:
        return au.grapplerobotics.interfaces.LaserCanInterface.RangingMode.SHORT;
      case MEDIUM:
        return au.grapplerobotics.interfaces.LaserCanInterface.RangingMode
            .SHORT; // LaserCAN doesn't distinguish MEDIUM.
      case LONG:
        return au.grapplerobotics.interfaces.LaserCanInterface.RangingMode.LONG;
      default:
        return null;
    }
  }

  /**
   * Converts the current distance mode to a PWF (Playing With Fusion) compatible ranging mode.
   *
   * @return The corresponding PWF RangingMode (Short, Medium, Long), or null if no valid mode.
   */
  public RangingMode asPWF() {
    switch (currentMode) {
      case SHORT:
        return RangingMode.Short;
      case MEDIUM:
        return RangingMode.Medium;
      case LONG:
        return RangingMode.Long;
      default:
        return null;
    }
  }

  /**
   * Converts a PWF RangingMode to a corresponding DistanceMode.
   *
   * @param mode The PWF RangingMode (Short, Medium, Long).
   * @return A new DistanceMode instance corresponding to the PWF RangingMode.
   */
  public DistanceMode fromPWF(RangingMode mode) {
    switch (mode) {
      case Short:
        return new DistanceMode(modes.SHORT);
      case Medium:
        return new DistanceMode(modes.MEDIUM);
      case Long:
        return new DistanceMode(modes.LONG);
      default:
        return null;
    }
  }

  /**
   * Converts a LaserCAN RangingMode to a corresponding DistanceMode.
   *
   * @param mode The LaserCAN RangingMode (SHORT, MEDIUM, LONG).
   * @return A new DistanceMode instance corresponding to the LaserCAN RangingMode.
   */
  public DistanceMode fromLaserCAN(
      au.grapplerobotics.interfaces.LaserCanInterface.RangingMode mode) {
    switch (mode) {
      case SHORT:
        return new DistanceMode(modes.SHORT);
      case LONG:
        return new DistanceMode(modes.LONG);
      default:
        return null;
    }
  }
}
