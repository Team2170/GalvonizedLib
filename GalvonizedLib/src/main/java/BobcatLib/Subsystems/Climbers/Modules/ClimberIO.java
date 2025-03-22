package BobcatLib.Subsystems.Climbers.Modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for controlling and monitoring the climber mechanism of a robot. */
public interface ClimberIO {

  /**
   * Data structure for storing inputs related to the climber mechanism.
   *
   * <p>This is used for AdvantageKit logging.
   */
  @AutoLog
  public static class ClimberIOInputs {
    /** The position of the climber motor represented as a double . */
    public double climberMotorPosition = 0;
  }

  /**
   * Configures the climber subsystem.
   *
   * <p>This method is called to initialize or reset the climber configuration to default values.
   */
  public default void configClimber() {}

  /**
   * Updates the input values of the climber for logging or telemetry purposes.
   *
   * @param inputs The {@link ClimberIOInputs} object containing updated input data.
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the motor output percentage for the climber.
   *
   * @param percent The desired motor output as a percentage (-1.0 to 1.0).
   */
  public default void setPercentOut(double percent) {}

  /**
   * Commands the climber to hold a specific position.
   *
   * @param rot The desired position to hold, in rotations.
   */
  public default void holdPos(double rot) {}

  /** Stops the climber mechanism by setting motor output to zero. */
  public default void stop() {}

  /**
   * Retrieves the current position of the climber mechanism.
   *
   * @return The current position as a {@link Rotation2d}.
   */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }
  /**
   * Sets the climber to a specified position.
   *
   * @param position The target position as a {@link Rotation2d} object.
   */
  public default void setPosition(Rotation2d position) {}
}
