package BobcatLib.Subsystems.Climbers;

import BobcatLib.Subsystems.Climbers.Modules.BaseClimber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the climber subsystem, responsible for managing the climber's behavior and
 * periodically updating its inputs.
 */
public class ClimberSubsystem extends SubsystemBase {
  private final BaseClimber climberModule;
  private String name = "";

  /**
   * Constructs a new {@code ElevatorSubsystem}.
   *
   * @param name The name of the subsystem, used for identification or debugging.
   * @param climberModule The {@link BaseClimber} implementation for controlling the elevator
   *     hardware.
   */
  public ClimberSubsystem(String name, BaseClimber climberModule) {
    this.name = name;
    this.climberModule = climberModule;
  }

  /** Periodically updates the elevator subsystem inputs. Called once per scheduler run. */
  @Override
  public void periodic() {
    climberModule.periodic();
  }

  /**
   * Sets the elevator motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public void setPercentOut(double percent) {
    climberModule.setPercentOut(percent);
  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d} object.
   */
  public void moveClimber(Rotation2d position) {
    climberModule.moveClimber(position);
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    climberModule.stop();
  }
}
