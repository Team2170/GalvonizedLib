package BobcatLib.Subsystems.Elevators.Modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorModuleIO {

  /**
   * Data structure for storing inputs related to the climber mechanism.
   *
   * <p>This is used for AdvantageKit logging.
   */
  @AutoLog
  public static class ElevatorIOInputs {
    /** The position of the climber motor represented as a double . */
    public double elevatorPosition = 0;

    public double currentSetPoint = 0;
  }

  /**
   * Moves the elevator to a specified setpoint.
   *
   * @param setPoint The target position for the elevator.
   */
  public default void moveElevator(Rotation2d setPoint) {}

  /**
   * Runs the elevator at a specified velocity.
   *
   * @param velocity The velocity at which to run the elevator.
   */
  public default void runElevator(double velocity) {}

  /** Stops the elevator motor immediately. */
  public default void stop() {}

  /**
   * Gets the current position of the elevator.
   *
   * @return The elevator position as a Rotation2d object.
   */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  /**
   * Updates the input values for the elevator module.
   *
   * @param inputs Elevator input structure containing position and setpoint data.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the current setpoint for the elevator position.
   *
   * @param rotations The setpoint in rotations.
   */
  public default void setCurrentSetPoint(double rotations) {}

  /**
   * Runs the elevator using a specified voltage.
   *
   * @param volts The voltage to apply to the motor.
   */
  public default void runVoltage(Voltage volts) {}
}
