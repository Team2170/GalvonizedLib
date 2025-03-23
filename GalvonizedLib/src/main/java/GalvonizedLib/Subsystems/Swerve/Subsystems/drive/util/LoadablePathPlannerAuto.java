package GalvonizedLib.Subsystems.Swerve.Subsystems.drive.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents a loadable path planner auto configuration with a name, command, and default status.
 * This class is immutable, ensuring thread-safety and consistent behavior.
 */
public final class LoadablePathPlannerAuto {
  private final String name;
  private final Command command;
  private final boolean isDefault;

  /**
   * Constructs a new {@code LoadablePathPlannerAutos} instance.
   *
   * @param name the name of the auto configuration; must not be null.
   * @param command the command associated with the auto configuration; must not be null.
   * @param isDefault indicates whether this auto configuration is the default.
   * @throws IllegalArgumentException if {@code name} or {@code command} is null.
   */
  public LoadablePathPlannerAuto(String name, Command command, boolean isDefault) {
    if (name == null || command == null) {
      throw new IllegalArgumentException("Arguments 'name' and 'command' cannot be null");
    }
    this.name = name;
    this.command = command;
    this.isDefault = isDefault;
  }

  /**
   * Gets the name of this auto configuration.
   *
   * @return the name of the auto configuration.
   */
  public String getName() {
    return name;
  }

  /**
   * Gets the command associated with this auto configuration.
   *
   * @return the command associated with the auto configuration.
   */
  public Command getCommand() {
    return command;
  }

  /**
   * Checks whether this auto configuration is the default.
   *
   * @return {@code true} if this is the default auto configuration; {@code false} otherwise.
   */
  public boolean isDefault() {
    return isDefault;
  }
}
