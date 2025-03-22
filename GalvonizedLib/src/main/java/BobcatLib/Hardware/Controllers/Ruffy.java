package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper class for the Ruffy joystick controller that implements the {@link ControllerWrapper}
 * interface. This class provides standardized access to joystick axes and button triggers for the
 * Ruffy controller.
 */
public class Ruffy implements ControllerWrapper {
  /** The underlying joystick object for the Ruffy controller. */
  CommandJoystick ruffyJoystick;

  /** Axis index for forward/backward LeftY. */
  public final int LeftYAxis = 1;

  /** Axis index for side-to-side strafing. */
  public final int LeftXAxis = 0;

  public int topButtonIndex = 1;

  /**
   * Constructs a Ruffy joystick controller wrapper for the specified port.
   *
   * @param port the port the Ruffy joystick is connected to.
   */
  public Ruffy(int port) {
    ruffyJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for LeftY (forward/backward) control.
   *
   * @return the axis value for LeftY control.
   */
  public double getLeftYAxis() {
    return ruffyJoystick.getRawAxis(LeftYAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getLeftXAxis() {
    return ruffyJoystick.getRawAxis(LeftXAxis);
  }

  /**
   * Gets the axis value for RightY control.
   *
   * @return the axis value for RightY control.
   */
  public double getRightYAxis() {
    return ruffyJoystick.getRawAxis(LeftYAxis);
  }

  /**
   * Gets the axis value for RightX control.
   *
   * @return the axis value for RightX control.
   */
  public double getRightXAxis() {
    return ruffyJoystick.getRawAxis(LeftXAxis);
  }

  /**
   * Gets the trigger for the left trigger button.
   *
   * @return a {@link Trigger} object for the left trigger button.
   */
  public Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the right trigger button.
   *
   * @return a {@link Trigger} object for the right trigger button.
   */
  public Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the left bumper button.
   *
   * @return a {@link Trigger} object for the left bumper button.
   */
  public Trigger getLeftBumper() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the right bumper button.
   *
   * @return a {@link Trigger} object for the right bumper button.
   */
  public Trigger getRightBumper() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Triangle button.
   *
   * @return a {@link Trigger} object for the Triangle button.
   */
  public Trigger getYorTriangle() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Circle button.
   *
   * @return a {@link Trigger} object for the Circle button.
   */
  public Trigger getBorCircle() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Cross button.
   *
   * @return a {@link Trigger} object for the Cross button.
   */
  public Trigger getAorCross() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Square button.
   *
   * @return a {@link Trigger} object for the Square button.
   */
  public Trigger getXorSquare() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a {@link Trigger} object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a {@link Trigger} object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a {@link Trigger} object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a {@link Trigger} object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return new Trigger(() -> false);
  }

  public Trigger getTopButton() {
    return ruffyJoystick.button(topButtonIndex);
  }
}
