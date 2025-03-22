package BobcatLib.Hardware.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/** Represents an input axis with deadband and slew rate limiting. */
public class Axis {
  /** The raw input value. */
  public double input;
  /** The input value after applying deadband. */
  public double inputDeadband;
  /** The input value after applying slew rate limiting. */
  public double inputLimited;
  /** The slew rate limiter object. */
  private SlewRateLimiter limiter;

  /**
   * Constructs an Axis object with the given raw input.
   *
   * @param input The raw input value.
   */
  public Axis(double input, double stickDeadband) {
    this.input = input;
    this.inputDeadband = MathUtil.applyDeadband(this.input, stickDeadband);
    this.limiter = new SlewRateLimiter(3.0);
    this.inputLimited = limiter.calculate(this.input);
  }

  /**
   * Gets the input value after applying deadband.
   *
   * @return The input value after applying deadband.
   */
  public double getDeadband() {
    return inputDeadband;
  }

  /**
   * Gets the input value after applying slew rate limiting.
   *
   * @return The input value after applying slew rate limiting.
   */
  public double getLimited() {
    return inputLimited;
  }

  /**
   * Gets the raw input value.
   *
   * @return The raw input value.
   */
  public double getInput() {
    return input;
  }
}
