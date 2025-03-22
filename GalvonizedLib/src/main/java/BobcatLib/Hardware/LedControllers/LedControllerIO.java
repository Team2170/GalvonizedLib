package BobcatLib.Hardware.LedControllers;

import BobcatLib.Hardware.LedControllers.CANdleWrapper.CANdleState;

/**
 * Interface representing the I/O layer for controlling LED controllers. This interface defines the
 * necessary methods for updating LED states, handling animations, and checking for faults in LED
 * hardware.
 */
public interface LedControllerIO {

  /** Represents the I/O inputs for the LED controller, including the current state of the LEDs. */
  public class LedControllerIOInputs {
    /** The current state of the LED controller (e.g., OFF, TURNING_CW, etc.). */
    public CANdleState currState = CANdleState.OFF;
  }

  /**
   * Updates the inputs for the LED controller I/O layer. This method should be used to update the
   * current state of the LED controller.
   *
   * @param inputs The `LedControllerIOInputs` object that will be updated with the current state.
   */
  public default void updateInputs(LedControllerIOInputs inputs) {}

  /**
   * Sets the LED controller to the specified animation state. This method will persist the
   * animation state until explicitly set to another state. It does not automatically turn off the
   * LEDs.
   *
   * @param state The animation state to set for the LEDs.
   */
  public default void setLEDs(CANdleState state) {}

  /**
   * Sets the LED controller to the specified animation state and duration. The animation will play
   * for the specified duration before either stopping or transitioning to another state.
   *
   * @param state The animation state to set for the LEDs.
   * @param seconds The duration in seconds for which the animation will play.
   */
  public default void setLEDs(CANdleState state, double seconds) {}

  /**
   * Periodically called method to handle periodic updates for the LED controller. This method
   * should be called in the robot's periodic loop to maintain the LED behavior.
   */
  public default void periodic() {}

  /**
   * Checks for faults in the LED hardware. This method should be used to detect any issues or
   * errors with the LED controller.
   */
  public default void checkForFaults() {}
}
