package BobcatLib.Subsystems.Climbers.Modules;

import BobcatLib.Subsystems.Climbers.Modules.ClimberIO.ClimberIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;

public class BaseClimber {
  /** The logged inputs for the climber, used for telemetry and diagnostics. */
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  /** The climber's input-output interface for hardware interaction. */
  private ClimberIO io;

  /**
   * Constructs a {@code ClimberSubsystem} with the specified name and input-output interface.
   *
   * @param io The {@link ClimberIO} interface for hardware interaction.
   */
  public BaseClimber(ClimberIO io) {
    this.io = io;
  }

  /**
   * Periodically updates the climber's inputs and handles its control behavior.
   *
   * <p>This method is called regularly by the scheduler to update telemetry and control logic. It
   * also uses AdvantageKit's Logger to log the climber's inputs for diagnostics.
   */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the climber motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  /**
   * Moves the climber to the specified position.
   *
   * @param position The desired position of the climber as a {@link Rotation2d} object.
   */
  public void moveClimber(Rotation2d position) {
    io.setPosition(position);
  }

  /** Stops the climbers motor immediately. */
  public void stop() {
    io.stop();
  }
}
