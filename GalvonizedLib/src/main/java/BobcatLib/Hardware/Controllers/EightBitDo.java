package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for the EightBitDo controller using WPILib's CommandJoystick. This class provides
 * mappings for the controller's axes, buttons, and D-Pad.
 */
public class EightBitDo implements ControllerWrapper {
  private CommandJoystick ebdJoystick;

  /** Axis index for forward/backward LeftY. */
  public final int LeftYAxis = 0;

  /** Axis index for side-to-side strafing. */
  public final int LeftXAxis = 1;

  /** Axis index for RightX. */
  public final int RightXAxis = 4;

  /**
   * Constructs an EightBitDo controller wrapper with mappings for joystick axes and buttons.
   *
   * @param port the port the controller is connected to.
   */
  public EightBitDo(int port) {
    ebdJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for LeftY (forward/backward) control.
   *
   * @return the axis value for LeftY control.
   */
  public double getLeftYAxis() {
    return ebdJoystick.getRawAxis(LeftYAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getLeftXAxis() {
    return ebdJoystick.getRawAxis(LeftXAxis);
  }

  /**
   * Gets the axis value for RightX control.
   *
   * @return the axis value for RightX control.
   */
  public double getRightXAxis() {
    return ebdJoystick.getRawAxis(RightXAxis);
  }

  /**
   * Gets the trigger for the left button (button 9).
   *
   * @return a Trigger object for the left button.
   */
  public Trigger getLeftTrigger() {
    return ebdJoystick.button(9);
  }

  /**
   * Gets the trigger for the right button (button 10).
   *
   * @return a Trigger object for the right button.
   */
  public Trigger getRightTrigger() {
    return ebdJoystick.button(10);
  }

  /**
   * Gets the trigger for the left bumper (button 5).
   *
   * @return a Trigger object for the left bumper.
   */
  public Trigger getLeftBumper() {
    return ebdJoystick.button(5);
  }

  /**
   * Gets the trigger for the right bumper (button 6).
   *
   * @return a Trigger object for the right bumper.
   */
  public Trigger getRightBumper() {
    return ebdJoystick.button(6);
  }

  /**
   * Gets the trigger for the Y or Triangle button (button 3).
   *
   * @return a Trigger object for the Y or Triangle button.
   */
  public Trigger getYorTriangle() {
    return ebdJoystick.button(3);
  }

  /**
   * Gets the trigger for the B or Circle button (button 1).
   *
   * @return a Trigger object for the B or Circle button.
   */
  public Trigger getBorCircle() {
    return ebdJoystick.button(1);
  }

  /**
   * Gets the trigger for the A or Cross button (button 2).
   *
   * @return a Trigger object for the A or Cross button.
   */
  public Trigger getAorCross() {
    return ebdJoystick.button(2);
  }

  /**
   * Gets the trigger for the X or Square button (button 4).
   *
   * @return a Trigger object for the X or Square button.
   */
  public Trigger getXorSquare() {
    return ebdJoystick.button(4);
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a Trigger object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return ebdJoystick.povUp();
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a Trigger object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return ebdJoystick.povDown();
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a Trigger object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return ebdJoystick.povLeft();
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a Trigger object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return ebdJoystick.povRight();
  }

  public Trigger getTopButton() {
    return new Trigger(() -> false);
  }
}
