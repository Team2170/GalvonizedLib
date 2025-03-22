package BobcatLib.Subsystems.Elevators.Modules.Motors;

import BobcatLib.Hardware.Motors.Utility.CTRE.MotionMagicWrapper;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import com.ctre.phoenix6.controls.VoltageOut;

/**
 * Interface representing a base elevator motor with default implementations for motor control,
 * position tracking, and configuration.
 */
public interface BaseElevatorMotor {

  /** Configures the angle motor with default settings. */
  public default void configAngleMotor() {}

  /**
   * Sets the angle of the motor using the internal PID controller.
   *
   * @param rotations The desired angle in rotations.
   */
  public default void setAngle(double rotations) {}

  /**
   * Sets the motor control using a percentage output.
   *
   * @param percent The percentage of power to apply to the motor.
   */
  public default void setControl(double percent) {}

  /**
   * Sets the motor control using a VoltageOut object.
   *
   * @param output The voltage output control for the motor.
   */
  public default void setControl(VoltageOut output) {}

  /**
   * Gets the current position of the steer motor.
   *
   * @return The position of the steer motor in rotations.
   */
  public default double getPosition() {
    return 0.00;
  }

  /**
   * Gets the error in position of the steer motor.
   *
   * @return The position error in rotations.
   */
  public default double getErrorPosition() {
    return 0.00;
  }

  /**
   * Sets the absolute position of the steer motor.
   *
   * @param absolutePosition The desired absolute position in rotations.
   */
  public default void setPosition(double absolutePosition) {}

  /** Stops the motor safely. */
  public default void stopMotor() {}

  /**
   * Applies Motion Magic configuration to the elevator motor.
   *
   * @param mmConfigs The Motion Magic configuration wrapper.
   * @return A new BaseElevatorMotor instance with Motion Magic settings applied.
   */
  public default BaseElevatorMotor withMotionMagic(MotionMagicWrapper mmConfigs) {
    return null;
  }

  /**
   * Applies software limit settings to the elevator motor.
   *
   * @param limitsWrapper The software limit configuration wrapper.
   * @return A new BaseElevatorMotor instance with software limit settings applied.
   */
  public default BaseElevatorMotor withSoftwareLimit(SoftwareLimitWrapper limitsWrapper) {
    return null;
  }
}
