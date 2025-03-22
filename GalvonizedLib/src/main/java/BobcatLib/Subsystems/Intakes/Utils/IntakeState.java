package BobcatLib.Subsystems.Intakes.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the current state of the intake mechanism, including the pivot angle and roller
 * velocity. This class encapsulates the pivot angle (as a Rotation2d) and the roller velocity.
 */
public class IntakeState {
  private final Rotation2d pivotAngle;
  private final double rollerVelocity;

  /** Default constructor for IntakeState, setting the pivot angle to 0 and roller velocity to 0. */
  public IntakeState() {
    this.rollerVelocity = 0.00;
    this.pivotAngle = new Rotation2d();
  }

  /**
   * Constructor for IntakeState with a specified roller velocity. The pivot angle is set to 0 by
   * default.
   *
   * @param rollerVelocity The velocity of the intake roller in meters per second.
   */
  public IntakeState(double rollerVelocity) {
    this.rollerVelocity = rollerVelocity;
    this.pivotAngle = new Rotation2d();
  }

  /**
   * Constructor for IntakeState with specified roller velocity and pivot angle.
   *
   * @param rollerVelocity The velocity of the intake roller in meters per second.
   * @param pivotAngle The current pivot angle as a Rotation2d object.
   */
  public IntakeState(double rollerVelocity, Rotation2d pivotAngle) {
    this.pivotAngle = pivotAngle;
    this.rollerVelocity = rollerVelocity;
  }

  /**
   * Gets the pivot angle of the intake mechanism.
   *
   * @return The current pivot angle as a Rotation2d object.
   */
  public Rotation2d getPivotAngle() {
    return pivotAngle;
  }

  /**
   * Gets the velocity of the intake roller.
   *
   * @return The current roller velocity in meters per second.
   */
  public double getRollerVelocity() {
    return rollerVelocity;
  }

  /**
   * Returns a string representation of the current IntakeState.
   *
   * @return A string describing the IntakeState, including the pivot angle and roller velocity.
   */
  @Override
  public String toString() {
    return "IntakeState{" + "pivotAngle=" + pivotAngle + ", rollerVelocity=" + rollerVelocity + '}';
  }
}
