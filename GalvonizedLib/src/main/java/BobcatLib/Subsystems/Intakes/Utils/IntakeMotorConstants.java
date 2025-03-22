package BobcatLib.Subsystems.Intakes.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents configuration constants for an intake motor. This class is immutable, meaning its
 * state cannot be changed after creation.
 */
public final class IntakeMotorConstants {

  /** The upper limit of the motor's operational range. */
  private final double upperRange;

  /** The lower limit of the motor's operational range. */
  private final double lowerRange;

  /** Whether the motor is inverted (reversed). */
  private final boolean isMotorInverted;

  /** Whether the brake mode is enabled for the motor. */
  private final boolean isBrakeModeEnabled;

  /** The angular offset for the motor, in rotations. */
  private final Rotation2d angleOffset;

  /** Proportional gain for the motor's PID controller. */
  private final double kP;

  /** Integral gain for the motor's PID controller. */
  private final double kI;

  /** Derivative gain for the motor's PID controller. */
  private final double kD;

  /** The circumference of the mechanism driven by the motor. */
  private final double mechanismCircumference;

  /** The unique identifier for the motor. */
  private final int motorId;

  /** Whether the motor uses an absolute encoder for position feedback. */
  private final boolean useAbsEncoder;

  /** The unique identifier for the absolute encoder (if used). */
  private final int absEncoderId;

  /**
   * Constructs an instance with essential motor configuration.
   *
   * @param motorId The unique identifier for the motor.
   * @param kP Proportional gain for the PID controller.
   * @param kI Integral gain for the PID controller.
   * @param kD Derivative gain for the PID controller.
   */
  public IntakeMotorConstants(int motorId, double kP, double kI, double kD) {
    this(motorId, kP, kI, kD, false, 0.0, 0, 0.0, 0.0, 0.0, false, true);
  }

  /**
   * Constructs an instance with all motor configuration parameters.
   *
   * @param motorId The unique identifier for the motor.
   * @param kP Proportional gain for the PID controller.
   * @param kI Integral gain for the PID controller.
   * @param kD Derivative gain for the PID controller.
   * @param useAbsEncoder Whether the motor uses an absolute encoder.
   * @param angleOffset The angular offset for the motor, in rotations.
   * @param absEncoderId The unique identifier for the absolute encoder.
   * @param mechanismCircumference The circumference of the mechanism driven by the motor.
   * @param upperRange The upper limit of the motor's operational range.
   * @param lowerRange The lower limit of the motor's operational range.
   * @param isMotorInverted Whether the motor is inverted (reversed).
   * @param isBrakeModeEnabled Whether the brake mode is enabled for the motor.
   */
  public IntakeMotorConstants(
      int motorId,
      double kP,
      double kI,
      double kD,
      boolean useAbsEncoder,
      double angleOffset,
      int absEncoderId,
      double mechanismCircumference,
      double upperRange,
      double lowerRange,
      boolean isMotorInverted,
      boolean isBrakeModeEnabled) {
    this.motorId = motorId;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.useAbsEncoder = useAbsEncoder;
    this.angleOffset = Rotation2d.fromRotations(angleOffset);
    this.absEncoderId = absEncoderId;
    this.mechanismCircumference = mechanismCircumference;
    this.upperRange = upperRange;
    this.lowerRange = lowerRange;
    this.isMotorInverted = isMotorInverted;
    this.isBrakeModeEnabled = isBrakeModeEnabled;
  }

  /** Constructs an instance with default values. */
  public IntakeMotorConstants() {
    this(0, 0.0, 0.0, 0.0);
  }

  // Getters

  /**
   * @return The upper limit of the motor's operational range.
   */
  public double getUpperRange() {
    return upperRange;
  }

  /**
   * @return The lower limit of the motor's operational range.
   */
  public double getLowerRange() {
    return lowerRange;
  }

  /**
   * @return Whether the motor is inverted (reversed).
   */
  public boolean isMotorInverted() {
    return isMotorInverted;
  }

  /**
   * @return Whether the brake mode is enabled for the motor.
   */
  public boolean isBrakeModeEnabled() {
    return isBrakeModeEnabled;
  }

  /**
   * @return The angular offset for the motor, as a {@link Rotation2d}.
   */
  public Rotation2d getAngleOffset() {
    return angleOffset;
  }

  /**
   * @return Proportional gain for the PID controller.
   */
  public double getKP() {
    return kP;
  }

  /**
   * @return Integral gain for the PID controller.
   */
  public double getKI() {
    return kI;
  }

  /**
   * @return Derivative gain for the PID controller.
   */
  public double getKD() {
    return kD;
  }

  /**
   * @return The circumference of the mechanism driven by the motor.
   */
  public double getMechanismCircumference() {
    return mechanismCircumference;
  }

  /**
   * @return The unique identifier for the motor.
   */
  public int getMotorId() {
    return motorId;
  }

  /**
   * @return Whether the motor uses an absolute encoder.
   */
  public boolean isUseAbsEncoder() {
    return useAbsEncoder;
  }

  /**
   * @return The unique identifier for the absolute encoder (if used).
   */
  public int getAbsEncoderId() {
    return absEncoderId;
  }

  @Override
  public String toString() {
    return "IntakeMotorConstants{"
        + "upperRange="
        + upperRange
        + ", lowerRange="
        + lowerRange
        + ", isMotorInverted="
        + isMotorInverted
        + ", isBrakeModeEnabled="
        + isBrakeModeEnabled
        + ", angleOffset="
        + angleOffset
        + ", kP="
        + kP
        + ", kI="
        + kI
        + ", kD="
        + kD
        + ", mechanismCircumference="
        + mechanismCircumference
        + ", motorId="
        + motorId
        + ", useAbsEncoder="
        + useAbsEncoder
        + ", absEncoderId="
        + absEncoderId
        + '}';
  }
}
