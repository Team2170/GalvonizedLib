package BobcatLib.Subsystems.Climbers.Utils;

/**
 * Represents the configuration parameters for the climber system. This class holds constants for
 * motor settings such as range limits, inversion state, brake mode, and PID tuning constants. Once
 * initialized, the fields cannot be modified, making the class immutable.
 */
public class ClimberConstants {

  /** The name of the climber system. */
  public final String name;

  /** The upper limit of the motor's operational range. */
  private final double upperRange;

  /** The lower limit of the motor's operational range. */
  private final double lowerRange;

  /** Whether the motor is inverted (reversed). */
  private final boolean isMotorInverted;

  /** Whether the brake mode is enabled for the motor. */
  private final boolean isBrakeModeEnabled;

  /** Proportional gain for the motor's PID controller. */
  private final double kP;

  /**
   * Constructor for creating an immutable ClimberConstants object.
   *
   * @param name The name of the climber system.
   * @param upperRange The upper limit of the motor's operational range.
   * @param lowerRange The lower limit of the motor's operational range.
   * @param isMotorInverted Indicates whether the motor is inverted (reversed).
   * @param isBrakeModeEnabled Indicates whether the brake mode is enabled for the motor.
   * @param kP The proportional gain for the motor's PID controller.
   */
  public ClimberConstants(
      String name,
      double upperRange,
      double lowerRange,
      boolean isMotorInverted,
      boolean isBrakeModeEnabled,
      double kP) {
    this.name = name;
    this.upperRange = upperRange;
    this.lowerRange = lowerRange;
    this.isMotorInverted = isMotorInverted;
    this.isBrakeModeEnabled = isBrakeModeEnabled;
    this.kP = kP;
  }

  /**
   * @return The name of the intake subsystem.
   */
  public String getName() {
    return name;
  }

  /**
   * Returns the upper limit of the motor's operational range.
   *
   * @return The upper range limit as a double.
   */
  public double getUpperRange() {
    return upperRange;
  }

  /**
   * Returns the lower limit of the motor's operational range.
   *
   * @return The lower range limit as a double.
   */
  public double getLowerRange() {
    return lowerRange;
  }

  /**
   * Returns whether the motor is inverted (reversed).
   *
   * @return True if the motor is inverted, otherwise false.
   */
  public boolean isMotorInverted() {
    return isMotorInverted;
  }

  /**
   * Returns whether the brake mode is enabled for the motor.
   *
   * @return True if brake mode is enabled, otherwise false.
   */
  public boolean isBrakeModeEnabled() {
    return isBrakeModeEnabled;
  }

  /**
   * Returns the proportional gain (kP) used for the motor's PID controller.
   *
   * @return The proportional gain as a double.
   */
  public double getKP() {
    return kP;
  }

  /**
   * Returns a string representation of the ClimberConstants object, showing all configuration
   * parameters.
   *
   * @return A string representation of the climber system's configuration.
   */
  @Override
  public String toString() {
    return "ClimberConstants{"
        + "name='"
        + name
        + '\''
        + ", upperRange="
        + upperRange
        + ", lowerRange="
        + lowerRange
        + ", isMotorInverted="
        + isMotorInverted
        + ", isBrakeModeEnabled="
        + isBrakeModeEnabled
        + ", kP="
        + kP
        + '}';
  }
}
