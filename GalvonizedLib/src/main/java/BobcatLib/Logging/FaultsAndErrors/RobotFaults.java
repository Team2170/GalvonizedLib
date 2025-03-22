package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages faults and alerts for the robot. This class is responsible for creating, activating, and
 * deactivating alerts related to robot faults.
 */
public class RobotFaults implements FaultsWrapper {

  /** A list of alerts triggered for faults in the robot. */
  public List<Alert> alerts = new ArrayList<Alert>();

  /** Constructs a new RobotFaults instance. */
  public RobotFaults() {}

  /**
   * Creates a new alert with a specified message and severity level, and adds it to the list of
   * alerts.
   *
   * @param msg The message describing the fault.
   * @param level The severity level of the alert.
   */
  public void createAlert(String msg, AlertType level) {
    Alert alert = new Alert("Robot", msg, level);
    alerts.add(alert);
  }

  /**
   * Activates an alert, marking it as triggered, and logs the alert for the robot.
   *
   * @param alert The alert to activate.
   */
  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("Robot");
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
}
