package BobcatLib.Subsystems.Climbers.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the state of the climber system, including its rotations. This class is immutable;
 * once the object is created, its state cannot be changed.
 */
public class ClimberState {

  /** The current rotations of the climber. */
  private final Rotation2d climberRotations;

  /** Default constructor that initializes the climber state with a zero rotation. */
  public ClimberState() {
    this.climberRotations = new Rotation2d();
  }

  /**
   * Constructor that initializes the climber state with the provided rotation.
   *
   * @param climberRotations The current rotations of the climber.
   */
  public ClimberState(Rotation2d climberRotations) {
    this.climberRotations = climberRotations;
  }

  /**
   * Gets the current rotation of the climber.
   *
   * @return The current climber rotations as a Rotation2d object.
   */
  public Rotation2d getClimberRotations() {
    return climberRotations;
  }

  /**
   * Returns a string representation of the ClimberState object.
   *
   * @return A string representation of the climber state.
   */
  @Override
  public String toString() {
    return "ClimberState{" + "climberRotations=" + climberRotations + '}';
  }
}
