package BobcatLib.Hardware.Motors;

public class OptionalConfigRev {
  /**
   * The maximum current limit (in Amps) for the motor's supply. This value ensures that the motor
   * does not draw more current than the specified limit, helping to prevent motor damage. A value
   * of 0 means no current limit is applied.
   */
  public int SupplyCurrentLimit = 0;

  /**
   * Conversion factor for velocity used in drive control. This factor is applied to convert a
   * measured velocity (e.g., from a sensor) into a usable value for motor control. It helps scale
   * the velocity to match the motor's performance and system requirements.
   */
  public double driveConversionVelocityFactor = 0.00;

  /**
   * Conversion factor for position used in drive control. This factor is applied to convert a
   * measured position (e.g., from an encoder) into a usable value for motor control. It helps scale
   * the position to match the motor's performance and system requirements.
   */
  public double driveConversionPositionFactor = 0.00;

  /**
   * Ramp rate for open-loop motor control, which dictates how quickly the motor ramps up or down
   * its speed in an open-loop system. A higher value results in faster ramping, while a lower value
   * provides smoother, slower ramping.
   */
  public double openLoopRamp = 0.00;

  /**
   * Ramp rate for closed-loop motor control, which dictates how quickly the motor ramps up or down
   * its speed in a closed-loop system. A higher value results in faster ramping, while a lower
   * value provides smoother, slower ramping.
   */
  public double closedLoopRamp = 0.00;
}
