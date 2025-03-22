package BobcatLib.Subsystems.Climbers.Utils.Parser;

/**
 * Represents the configuration and settings for a climber mechanism.
 *
 * <p>This class is intended to store the climber's properties, limits, and control parameters in a
 * structured and serializable format (e.g., JSON).
 */
public class ClimberJson {
  /** Represents the main attributes of the elevator. */
  public class ClimberMainJson {
    /** The name of the elevator system. */
    public String name = "";
  }

  /** Represents the operational limits of the elevator. */
  public class ClimberLimitsJson {
    /** The upper limit for the elevator's position. */
    public double upperLimits = 0;

    /** The lower limit for the elevator's position. */
    public double lowerLimits = 0;
  }

  /** An instance of {@link ClimberMainJson}, representing the Climbers's main configuration. */
  public ClimberMainJson elevator = new ClimberMainJson();

  /**
   * Indicates whether the climber motor direction is inverted. If true, the motor runs in the
   * opposite direction of the default configuration.
   */
  public boolean inverted = false;

  /**
   * Indicates whether the climber motor is in brake mode.
   *
   * <p>When true, the motor resists motion when power is not applied.
   */
  public boolean climberMotorBreakMode = true;

  public int motorId = 0;

  public double kP = 0.0;

  /**
   * An instance of {@link ClimberLimitsJson}, representing the operational limits of the climber.
   */
  public ClimberLimitsJson limits = new ClimberLimitsJson();
}
