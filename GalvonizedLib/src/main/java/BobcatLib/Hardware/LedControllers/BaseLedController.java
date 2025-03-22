package BobcatLib.Hardware.LedControllers;

import BobcatLib.Hardware.LedControllers.LedControllerIO.LedControllerIOInputs;

/**
 * Represents a base LED controller that interacts with an LED controller hardware interface. It
 * periodically updates the LED controller inputs and checks for faults.
 */
public class BaseLedController {
  private final LedControllerIO io;
  private final LedControllerIOInputs inputs = new LedControllerIOInputs();
  private final String name;

  /**
   * Constructs a BaseLedController instance with the specified name and LED controller interface.
   *
   * @param name The name of the LED controller.
   * @param io The LED controller input/output interface to interact with the hardware.
   */
  public BaseLedController(String name, LedControllerIO io) {
    this.io = io;
    this.name = name;
  }

  /**
   * Periodically updates the LED controller inputs by calling the corresponding method from the I/O
   * interface.
   */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Checks for faults in the LED controller by calling the corresponding method from the I/O
   * interface.
   */
  public void checkForFaults() {
    io.checkForFaults();
  }
}
