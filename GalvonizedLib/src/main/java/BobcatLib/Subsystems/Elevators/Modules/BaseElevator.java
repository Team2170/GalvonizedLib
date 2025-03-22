package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

/**
 * Base class for controlling an elevator subsystem. Manages elevator movement, setpoints, and
 * logging.
 */
public class BaseElevator {
  private final ElevatorModuleIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private Rotation2d currentSetPoint = new Rotation2d();

  /**
   * Constructs a BaseElevator instance with the given elevator module IO.
   *
   * @param io The elevator module interface for hardware interactions.
   */
  public BaseElevator(ElevatorModuleIO io) {
    this.io = io;
  }

  /** Updates the elevator state by setting the current setpoint and logging inputs. */
  public void update() {
    io.setCurrentSetPoint(currentSetPoint.getRotations());
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/Module", inputs);
  }

  /**
   * Moves the elevator to the next setpoint.
   *
   * @param setPoints The wrapper containing multiple setpoints.
   */
  public void moveElevatorToNext(SetPointWrapper setPoints) {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    Rotation2d nextPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(1));
    currentSetPoint = nextPosition;
  }

  /**
   * Moves the elevator to the previous setpoint.
   *
   * @param setPoints The wrapper containing multiple setpoints.
   */
  public void moveElevatorToPrevious(SetPointWrapper setPoints) {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    Rotation2d previousPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(0));
    currentSetPoint = previousPosition;
  }

  /** Holds the elevator at its current position. */
  public void holdPosition() {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    currentSetPoint = currentPosition;
  }

  /**
   * Gets the current setpoint of the elevator.
   *
   * @return The current setpoint as a Rotation2d object.
   */
  public Rotation2d getCurrentSetPoint() {
    return currentSetPoint;
  }

  /**
   * Runs the elevator at a specified velocity.
   *
   * @param velocity The velocity at which to run the elevator.
   */
  public void runElevator(double velocity) {
    io.runElevator(velocity);
  }

  /** Runs the elevator to the current setpoint. */
  public void runElevator() {
    io.moveElevator(currentSetPoint);
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    io.stop();
  }

  /**
   * Runs the elevator using a specified voltage.
   *
   * @param volts The voltage to apply to the motor.
   */
  public void runVoltage(Voltage volts) {
    io.runVoltage(volts);
  }
}
