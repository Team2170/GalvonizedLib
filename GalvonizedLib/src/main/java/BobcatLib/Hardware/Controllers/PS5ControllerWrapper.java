package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper class for the PS5 controller that extends {@link CommandPS5Controller} and implements
 * the {@link ControllerWrapper} interface. This class provides standardized access to joystick axes
 * and button triggers for a PS5 controller.
 */
public class PS5ControllerWrapper extends CommandPS5Controller implements ControllerWrapper {
  /** Axis index for forward/backward LeftY. */
  public final int LeftYAxis = PS5Controller.Axis.kLeftY.value;

  /** Axis index for side-to-side strafing. */
  public final int LeftXAxis = PS5Controller.Axis.kLeftX.value;

  /** Axis index for RightX. */
  public final int RightXAxis = PS5Controller.Axis.kRightX.value;

  /**
   * Constructs a PS5 controller wrapper for the specified port.
   *
   * @param port the port the PS5 controller is connected to.
   */
  public PS5ControllerWrapper(int port) {
    super(port);
  }

  /**
   * Gets the axis value for LeftY (forward/backward) control.
   *
   * @return the axis value for LeftY control.
   */
  public double getLeftYAxis() {
    return super.getRawAxis(LeftYAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getLeftXAxis() {
    return super.getRawAxis(LeftXAxis);
  }

  /**
   * Gets the axis value for RightX control.
   *
   * @return the axis value for RightX control.
   */
  public double getRightXAxis() {
    return super.getRawAxis(RightXAxis);
  }

  /**
   * Gets the trigger for the left trigger (L2).
   *
   * @return a {@link Trigger} object for the left trigger.
   */
  public Trigger getLeftTrigger() {
    return super.L2();
  }

  /**
   * Gets the trigger for the right trigger (R2).
   *
   * @return a {@link Trigger} object for the right trigger.
   */
  public Trigger getRightTrigger() {
    return super.R2();
  }

  /**
   * Gets the trigger for the left bumper (L1).
   *
   * @return a {@link Trigger} object for the left bumper.
   */
  public Trigger getLeftBumper() {
    return super.L1();
  }

  /**
   * Gets the trigger for the right bumper (R1).
   *
   * @return a {@link Trigger} object for the right bumper.
   */
  public Trigger getRightBumper() {
    return super.R1();
  }

  /**
   * Gets the trigger for the Triangle button.
   *
   * @return a {@link Trigger} object for the Triangle button.
   */
  public Trigger getYorTriangle() {
    return super.triangle();
  }

  /**
   * Gets the trigger for the Circle button.
   *
   * @return a {@link Trigger} object for the Circle button.
   */
  public Trigger getBorCircle() {
    return super.circle();
  }

  /**
   * Gets the trigger for the Cross button.
   *
   * @return a {@link Trigger} object for the Cross button.
   */
  public Trigger getAorCross() {
    return super.cross();
  }

  /**
   * Gets the trigger for the Square button.
   *
   * @return a {@link Trigger} object for the Square button.
   */
  public Trigger getXorSquare() {
    return super.square();
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a {@link Trigger} object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return super.povUp();
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a {@link Trigger} object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return super.povDown();
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a {@link Trigger} object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return super.povLeft();
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a {@link Trigger} object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return super.povRight();
  }

  public Trigger getTopButton() {
    return new Trigger(() -> false);
  }
}
