package BobcatLib.Subsystems.Intakes.Modules;

import BobcatLib.Subsystems.Intakes.Parser.IntakeJson;
import BobcatLib.Subsystems.Intakes.Utils.IntakeState;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the Intake Module Input/Output (IO) layer. Defines methods to interact with and
 * control the intake mechanism, including setting states, managing positions, and obtaining sensor
 * data.
 */
public interface IntakeModuleIO {

  /**
   * Inner class representing the inputs to the intake module. Annotated with @AutoLog for automatic
   * logging.
   */
  @AutoLog
  public static class IntakeIOInputs {
    // Current pivot angle of the intake mechanism.
    public Rotation2d pivotAngle = new Rotation2d();

    // Current roller velocity of the intake mechanism.
    public double rollerVelocity = 0.0;
  }

  /**
   * Updates the input values of the intake module.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the overall state of the intake mechanism.
   *
   * @param state The desired state of the intake mechanism.
   */
  public default void setIntakeState(IntakeState state) {}

  /**
   * Sets the intake mechanism to operate in single mode.
   *
   * @param state The desired state for single mode.
   */
  public default void setSingleModeState(IntakeState state) {}

  /**
   * Sets the intake mechanism to operate in pivoting mode.
   *
   * @param state The desired state for pivoting mode.
   */
  public default void setPivotingModeState(IntakeState state) {}

  /**
   * Sets the pivot angle of the intake mechanism.
   *
   * @param rotations The desired angle in rotations.
   */
  public default void setAngle(double rotations) {}

  /**
   * Sets the velocity of the intake roller.
   *
   * @param speedInMPS The desired velocity in meters per second.
   * @param mechanismCircumference The circumference of the roller mechanism.
   * @param isOpen Whether the roller should be operating in open-loop mode.
   */
  public default void setVelocity(
      double speedInMPS, double mechanismCircumference, boolean isOpen) {}

  /**
   * Holds the pivot mechanism at a specific position.
   *
   * @param rot The desired position in rotations.
   */
  public default void holdPos(double rot) {}

  /** Stops all movement of the intake mechanism. */
  public default void stop() {}

  /**
   * Gets the current state of the intake mechanism.
   *
   * @return The current state.
   */
  public default IntakeState getState() {
    return new IntakeState();
  }

  /**
   * Gets the current pivot position of the intake mechanism.
   *
   * @return The current pivot angle as a Rotation2d object.
   */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  /**
   * Gets the current velocity of the intake roller.
   *
   * @return The current velocity in meters per second.
   */
  public default double getVelocity() {
    return 0.00;
  }

  public default void loadConfigurationFromFile(IntakeJson intakeJson) {}
}
