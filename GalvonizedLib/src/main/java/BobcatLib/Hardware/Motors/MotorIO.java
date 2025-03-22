package BobcatLib.Hardware.Motors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The MotorIO interface defines the methods and data structures for interacting with motor input
 * and output operations. It provides default implementations for basic motor functionalities,
 * enabling easier integration with various motor types.
 */
public interface MotorIO {

  /** Represents the inputs for the motor sensor. */
  @AutoLog
  public class MotorIOInputs {
    /** The current position of the motor as a Rotation2d object. */
    public Rotation2d motorPosition = new Rotation2d();

    /** The current velocity of the motor in meters per second. */
    public double motorVelocity = 0.00;

    /** Indicates whether the motor is in a faulted state. */
    public boolean faulted = false;
  }

  /**
   * Updates the motor inputs based on external sources.
   *
   * @param inputs The MotorIOInputs object to update with the latest sensor data.
   */
  public default void updateInputs(MotorIOInputs inputs) {}

  /**
   * Retrieves the time difference used for calculations.
   *
   * @return The time difference in seconds.
   */
  public default double getTimeDiff() {
    return 1.0;
  }

  /**
   * Performs periodic updates using a KrakenMotor instance.
   *
   * @param motor The KrakenMotor instance to use for updates.
   */
  public default void periodic(KrakenMotor motor) {}

  /**
   * Performs periodic updates using a FalconMotor instance.
   *
   * @param motor The FalconMotor instance to use for updates.
   */
  public default void periodic(FalconMotor motor) {}

  /** Performs periodic updates. */
  public default void periodic() {}

  /**
   * Retrieves the current position of the motor.
   *
   * @return The motor's position as a Rotation2d object.
   */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor's velocity in meters per second.
   */
  public default double getVelocity() {
    return 0;
  }

  /**
   * Sets the motor speed.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public default void setSpeed(double speedInMPS, boolean isOpenLoop) {}

  /**
   * Sets the motor speed with additional parameters.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public default void setSpeed(
      double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {}

  /**
   * Sets the motor angle in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public default void setAngle(double angleInRotations) {}

  /**
   * Sets the motor control voltage.
   *
   * @param volts The voltage to apply to the motor.
   */
  public default void setControl(double volts) {}

  /** Stops the motor. */
  public default void stopMotor() {}

  /** Checks for motor faults. */
  public default void checkForFaults() {}

  /**
   * Retrieves the CAN ID of the motor.
   *
   * @return The motor's CAN ID.
   */
  public default int getCanId() {
    return 0;
  }
}
