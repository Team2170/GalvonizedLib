package BobcatLib.Hardware.Motors.Utility;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.revrobotics.spark.config.LimitSwitchConfig;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Wrapper class for software limit switch configurations, supporting both CTRE and REV hardware.
 */
public class SoftwareLimitWrapper {

  /** Enum to define the type of software limit. */
  public enum SoftwareLimitType {
    BOTH,
    LOWER,
    UPPER
  }

  private SoftwareLimitType mode;
  private double lowerLimit, upperLimit;
  private boolean enableLower, enableUpper;

  /**
   * Constructor for a single software limit.
   *
   * @param limit the value of the limit (either lower or upper).
   * @param type the type of software limit (LOWER or UPPER).
   */
  public SoftwareLimitWrapper(double limit, SoftwareLimitType type) {
    this.mode = type;
    if (type == SoftwareLimitType.LOWER) {
      lowerLimit = limit;
      enableLower = true;
      enableUpper = false;
    } else {
      upperLimit = limit;
      enableLower = false;
      enableUpper = true;
    }
  }

  /**
   * Constructor for both lower and upper software limits.
   *
   * @param lower the lower limit value.
   * @param upper the upper limit value.
   * @param type the type of software limit (BOTH).
   */
  public SoftwareLimitWrapper(double lower, double upper, SoftwareLimitType type) {
    this.lowerLimit = lower;
    this.upperLimit = upper;
    this.mode = type;
    this.enableLower = true;
    this.enableUpper = true;
  }

  /**
   * Converts the current configuration to a CTRE-compatible SoftwareLimitSwitchConfigs object.
   *
   * @return the configured SoftwareLimitSwitchConfigs instance.
   */
  public SoftwareLimitSwitchConfigs asCTRE() {
    SoftwareLimitSwitchConfigs softLimitThresh = new SoftwareLimitSwitchConfigs();
    softLimitThresh.withReverseSoftLimitThreshold(lowerLimit);
    softLimitThresh.withReverseSoftLimitEnable(enableLower);
    softLimitThresh.withForwardSoftLimitThreshold(lowerLimit);
    softLimitThresh.withForwardSoftLimitEnable(enableUpper);
    return softLimitThresh;
  }

  /**
   * Converts the current configuration to a REV-compatible LimitSwitchConfig object.
   *
   * @return the configured LimitSwitchConfig instance.
   */
  public LimitSwitchConfig asRev() {
    LimitSwitchConfig config = new LimitSwitchConfig();
    config.forwardLimitSwitchEnabled(enableUpper);
    config.reverseLimitSwitchEnabled(enableLower);
    return config;
  }

  /**
   * Gets the upper limit as a Rotation2d object.
   *
   * @return the upper limit converted to a Rotation2d instance.
   */
  public Rotation2d getUpperLimit() {
    return Rotation2d.fromRotations(upperLimit);
  }

  /**
   * Gets the lower limit as a Rotation2d object.
   *
   * @return the lower limit converted to a Rotation2d instance.
   */
  public Rotation2d getLowerLimit() {
    return Rotation2d.fromRotations(lowerLimit);
  }
}
