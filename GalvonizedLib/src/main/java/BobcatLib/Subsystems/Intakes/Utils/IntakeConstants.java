package BobcatLib.Subsystems.Intakes.Utils;

/**
 * Represents configuration constants for an intake subsystem. This class is immutable, meaning its
 * state cannot be changed after creation.
 */
public final class IntakeConstants {

  /** The name of the intake subsystem. */
  private final String name;

  /** Motor constants for the pivot motor. */
  private final IntakeMotorConstants pivotMotorConstants;

  /** Motor constants for the roller motor. */
  private final IntakeMotorConstants rollerMotorConstants;

  /** Indicates whether a pivot motor is used in the subsystem. */
  private final boolean usePivotMotor;

  /**
   * Constructor for an IntakeConstants instance without pivot motor usage.
   *
   * @param name The name of the intake subsystem.
   * @param rollerMotorConstants The motor constants for the roller.
   */
  public IntakeConstants(String name, IntakeMotorConstants rollerMotorConstants) {
    this.name = name;
    this.rollerMotorConstants = rollerMotorConstants;
    this.pivotMotorConstants = new IntakeMotorConstants(); // Default value
    this.usePivotMotor = false; // Immutable state
  }

  /**
   * Constructor for an IntakeConstants instance with pivot motor usage.
   *
   * @param name The name of the intake subsystem.
   * @param rollerMotorConstants The motor constants for the roller.
   * @param pivotMotorConstants The motor constants for the pivot motor.
   */
  public IntakeConstants(
      String name,
      IntakeMotorConstants rollerMotorConstants,
      IntakeMotorConstants pivotMotorConstants) {
    this.name = name;
    this.rollerMotorConstants = rollerMotorConstants;
    this.pivotMotorConstants = pivotMotorConstants;
    this.usePivotMotor = true; // Immutable state
  }

  /**
   * @return The name of the intake subsystem.
   */
  public String getName() {
    return name;
  }

  /**
   * @return The motor constants for the pivot motor.
   */
  public IntakeMotorConstants getPivotMotorConstants() {
    return pivotMotorConstants;
  }

  /**
   * @return The motor constants for the roller motor.
   */
  public IntakeMotorConstants getRollerMotorConstants() {
    return rollerMotorConstants;
  }

  /**
   * @return Whether a pivot motor is used in the subsystem.
   */
  public boolean isUsePivotMotor() {
    return usePivotMotor;
  }

  /**
   * Provides a string representation of the IntakeConstants instance.
   *
   * @return A string representation of the instance, including field values.
   */
  @Override
  public String toString() {
    return "IntakeConstants{"
        + "name='"
        + name
        + '\''
        + ", pivotMotorConstants="
        + pivotMotorConstants.toString()
        + ", rollerMotorConstants="
        + rollerMotorConstants.toString()
        + ", usePivotMotor="
        + usePivotMotor
        + '}';
  }
}
