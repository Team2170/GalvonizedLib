package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * Handles fault management for SparkMax motor controllers. Provides functionality for creating and
 * managing alerts for various fault conditions.
 */
public class SparkMaxFaults implements FaultsWrapper {

  /** Enumeration of possible faults that can occur in the SparkMax motor controller. */
  public enum faults {
    Brownout,
    Overcurrent,
    MotorFault,
    SensorFault,
    Stall,
    kHasReset
  }

  /** Alert for a Brownout fault. */
  public static Alert BrownoutAlert;

  /** Alert for an Overcurrent fault. */
  public static Alert OvercurrentAlert;

  /** Alert for a Motor fault. */
  public static Alert MotorFaultAlert;

  /** Alert for a Sensor fault. */
  public static Alert SensorFaultAlert;

  /** Alert for a Stall fault. */
  public static Alert StallAlert;

  /** Alert for a controller reset (kHasReset). */
  public static Alert kHasResetAlert;

  /** ID of the SparkMax motor controller associated with these faults. */
  public int id;

  /**
   * Constructs a new instance of `SparkMaxFaults` for a given motor controller.
   *
   * @param id The unique identifier of the SparkMax motor controller.
   */
  public SparkMaxFaults(int id) {
    AlertType level = AlertType.INFO;
    BrownoutAlert = new Alert("SparkMax", "SparkMax " + id + " hardware fault occurred", level);
    OvercurrentAlert = new Alert("SparkMax", "SparkMax " + id + " has overcurrent", level);
    MotorFaultAlert = new Alert("SparkMax", "SparkMax " + id + " has motor fault", level);
    SensorFaultAlert = new Alert("SparkMax", "SparkMax " + id + " has sensor fault", level);
    StallAlert = new Alert("SparkMax", "SparkMax " + id + " has stalled", level);
    kHasResetAlert = new Alert("SparkMax", "SparkMax " + id + " has reset", level);
  }

  /**
   * Activates an alert, marking it as triggered and logs the alert with the associated SparkMax ID.
   *
   * @param alert The alert to activate.
   */
  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("SparkMax " + id);
  }

  /**
   * Activates an alert with a specific severity level.
   *
   * @param alert The alert to activate.
   * @param type The severity level to set for the alert.
   */
  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  /**
   * Deactivates an alert, marking it as resolved.
   *
   * @param alert The alert to deactivate.
   */
  public void disableAlert(Alert alert) {
    alert.set(false);
  }

  /**
   * logs the state and returns the alert state.
   *
   * @param key which represents the advantage scope key that is being written too.
   * @param value from getting if the hardware has an error.
   * @return the logged state.
   */
  public boolean LogError(String key, boolean value) {
    Logger.recordOutput("Alerts/SparkMax/" + id + "/" + key, value);
    return value;
  }
}
