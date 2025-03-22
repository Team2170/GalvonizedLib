package BobcatLib.Hardware.LedControllers;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import BobcatLib.Logging.FaultsAndErrors.CANdleFaultsWrapper;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A wrapper class for interacting with a CANdle LED controller. This class allows controlling LED
 * animations, checking for faults, and updating LED states.
 */
public class CANdleWrapper implements LedControllerIO {

  /** Enum representing the various states of the CANdle. */
  public enum CANdleState {
    OFF,
    TURNING_CW,
    TURNING_CCW,
    RESETPOSE,
    RESETGYRO,
    ALIGNING,
    ALIGNED
  }

  /** CANdle I/O inputs, representing the current state of the CANdle. */
  public static class CANdleIOInputs {
    /** Current state of the CANdle. */
    public CANdleState state = CANdleState.OFF;
  }

  /** Enum representing the built-in LED animation types. */
  public enum BuiltInAnimations {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  }

  /** Boolean flag to determine if the CANdle is enabled. */
  public boolean isEnabled = false;

  private CANdle leds;
  private CANdleState currState = CANdleState.OFF;
  private CANdleFaultsWrapper faults;
  private double seconds = 1;
  private Timer timer = new Timer();
  private double defaultTime = 1;
  /** Alert for detecting sensor issues with CANdle. */
  public Alert sensorAlert;

  private int ledCount;

  private CANDeviceDetails details;

  /**
   * Constructor for the CANdleWrapper, initializing the CANdle hardware with the specified device
   * details, LED count, and CAN bus.
   *
   * @param details The device details for the CANdle.
   * @param ledCount The number of LEDs to control.
   * @param canbus The CAN bus to use for communication with the device.
   */
  public CANdleWrapper(CANDeviceDetails details, int ledCount, String canbus) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.ledCount = ledCount;
    try {
      leds = new CANdle(id, canbus);
      faults = new CANdleFaultsWrapper(leds, id);
      isEnabled = true;
    } catch (Exception e) {
      // Handle exception and set alert for hardware failure
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("LED", "LEDs " + id + " hardware not found", level);
      sensorAlert.set(true);
    }
    timer.stop();
    timer.reset();
  }

  /**
   * Updates the inputs for the LED controller I/O layer with the current CANdle state.
   *
   * @param inputs The CANdleIOInputs object to update.
   */
  public void updateInputs(LedControllerIOInputs inputs) {
    inputs.currState = currState;
  }

  /**
   * Sets the LED state to the specified CANdle state. This method will not automatically turn off
   * the LEDs; it will persist until explicitly set to a different state.
   *
   * @param state The animation state to set on the LEDs.
   */
  public void setLEDs(CANdleState state) {
    timer.stop();
    timer.reset();
    if (!isEnabled) {
      return;
    }
    // Core animation logic
    if (state != currState) {
      leds.animate(null); // Wipe old state when setting new one
    }
    currState = state;
    switch (state) {
      case RESETPOSE: // Strobe gold
        leds.animate(new StrobeAnimation(255, 170, 0, 0, 0.25, ledCount));
        break;
      case RESETGYRO: // Strobe orangish
        leds.animate(new StrobeAnimation(245, 129, 66, 0, 0.25, ledCount));
        break;
      case ALIGNING: // Strobe white
        leds.animate(new StrobeAnimation(255, 255, 255, 255, 0.75, ledCount));
        break;
      case ALIGNED: // Solid blue
        leds.animate(new StrobeAnimation(0, 0, 255, 0, 1, ledCount));
        break;
      case OFF:
      default:
        leds.animate(null); // Turn off LEDs
        break;
    }
  }

  /**
   * Sets the LED state to the specified CANdle state and duration. This method allows specifying
   * how long the animation should play.
   *
   * @param state The animation state to set on the LEDs.
   * @param seconds The duration in seconds for the animation.
   */
  public void setLEDs(CANdleState state, double seconds) {
    setLEDs(state);
    this.seconds = seconds;
    timer.reset();
    timer.start();
  }

  /**
   * Periodically checks the timer and updates the LED state. This method is typically called in the
   * robot's periodic function.
   */
  public void periodic() {
    if (timer.hasElapsed(seconds)) {
      if (isEnabled) {
        setLEDs(CANdleState.OFF); // Turn off LEDs after the set duration
      }
      timer.stop();
      timer.reset();
      seconds = defaultTime; // Reset to default duration
    }
  }

  /**
   * Checks for any faults that may have occurred in the CANdle hardware. If any faults are
   * detected, they are logged or alerted.
   */
  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
