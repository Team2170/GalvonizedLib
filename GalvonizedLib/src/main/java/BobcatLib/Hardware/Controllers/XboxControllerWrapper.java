package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper class for the Xbox controller that implements the {@link ControllerWrapper} interface.
 * This class provides standardized access to joystick axes and button triggers for the Xbox
 * controller.
 */
public class XboxControllerWrapper extends CommandXboxController implements ControllerWrapper {
  /** Axis index for forward/backward LeftY. */
  public final int LeftYAxis = XboxController.Axis.kLeftY.value;

  /** Axis index for side-to-side strafing. */
  public final int LeftXAxis = XboxController.Axis.kLeftX.value;

  /** Axis index for RightX control. */
  public final int RightXAxis = XboxController.Axis.kRightX.value;

  /** Axis index for RightY control. */
  public final int RightYAxis = XboxController.Axis.kRightY.value;

  /**
   * Constructs an Xbox controller wrapper for the specified port. The wrapper includes access to
   * left and right joysticks, POV controls, and button mappings (Y, B, A, X).
   *
   * @param port the port the Xbox controller is connected to.
   */
  public XboxControllerWrapper(int port) {
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
   * Gets the axis value for RightY control.
   *
   * @return the axis value for RightY control.
   */
  public double getRightYAxis() {
    return super.getRawAxis(RightYAxis);
  }

  /**
   * Gets the trigger for the left trigger button.
   *
   * @return a {@link Trigger} object for the left trigger button.
   */
  public Trigger getLeftTrigger() {
    return super.leftTrigger(0.3);
  }

  /**
   * Gets the trigger for the right trigger button.
   *
   * @return a {@link Trigger} object for the right trigger button.
   */
  public Trigger getRightTrigger() {
    return super.rightTrigger(0.3);
  }

  /**
   * Gets the trigger for the left bumper button.
   *
   * @return a {@link Trigger} object for the left bumper button.
   */
  public Trigger getLeftBumper() {
    return super.leftBumper();
  }

  /**
   * Gets the trigger for the right bumper button.
   *
   * @return a {@link Trigger} object for the right bumper button.
   */
  public Trigger getRightBumper() {
    return super.rightBumper();
  }

  /**
   * Gets the trigger for the Y button.
   *
   * @return a {@link Trigger} object for the Y button.
   */
  public Trigger getYorTriangle() {
    return super.y();
  }

  /**
   * Gets the trigger for the B button.
   *
   * @return a {@link Trigger} object for the B button.
   */
  public Trigger getBorCircle() {
    return super.b();
  }

  /**
   * Gets the trigger for the A button.
   *
   * @return a {@link Trigger} object for the A button.
   */
  public Trigger getAorCross() {
    return super.a();
  }

  /**
   * Gets the trigger for the X button.
   *
   * @return a {@link Trigger} object for the X button.
   */
  public Trigger getXorSquare() {
    return super.x();
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
}
