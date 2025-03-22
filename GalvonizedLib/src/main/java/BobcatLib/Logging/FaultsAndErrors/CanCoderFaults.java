package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wrapper class for managing and detecting faults in a CANcoder device. Provides functionality to
 * monitor and alert on various fault conditions.
 */
public class CanCoderFaults implements FaultsWrapper {

  /** ID of the CANcoder device. */
  public int id;

  /** CANcoder hardware instance being monitored for faults. */
  public CANcoder encoder;

  /** Alert for under-voltage fault. */
  public static Alert underVoltAlert;

  /** Alert for magnet being out of range. */
  public static Alert magnetOutOfRangeAlert;

  /** Alert for temperature being out of the acceptable range. */
  public static Alert outOfTemperatureRangeAlert;

  /** Alert for hardware fault. */
  public static Alert HardwareAlert;

  /** Alert for unlicensed feature usage. */
  public static Alert UnlicensedFeatureInUseAlert;

  /** Alert for reboot during enable state. */
  public static Alert bootDuringEnableAlert;

  /**
   * Constructor for initializing the fault manager for a CANcoder device.
   *
   * @param enc The CANcoder instance to monitor.
   * @param id The ID of the CANcoder device.
   */
  public CanCoderFaults(CANcoder enc, int id) {
    this.id = id;
    this.encoder = enc;
    AlertType level = AlertType.INFO;

    underVoltAlert =
        new Alert(
            "Encoders", "CanAndCoder " + id + " is undervoltaged, potential brownout.", level);
    HardwareAlert = new Alert("Encoders", "CanAndCoder " + id + " has a hardware fault.", level);
    UnlicensedFeatureInUseAlert =
        new Alert("Encoders", "CanAndCoder " + id + " is using unlicensed.", level);
    bootDuringEnableAlert =
        new Alert("Encoders", "CanAndCoder " + id + " recently rebooted.", level);
    outOfTemperatureRangeAlert =
        new Alert("Encoders", "CanAndCoder " + id + " is too high or low", level);
    magnetOutOfRangeAlert =
        new Alert("Encoders", "CanAndCoder " + id + " magnet is out of range", level);
  }

  /**
   * Activates a specific alert and logs it with a message.
   *
   * @param alert The alert to activate.
   */
  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("CanCoder " + id);
  }

  /**
   * Activates a specific alert and sets its severity level.
   *
   * @param alert The alert to activate.
   * @param type The severity level of the alert.
   */
  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  /**
   * Disables a specific alert, indicating that the fault condition no longer exists.
   *
   * @param alert The alert to disable.
   */
  public void disableAlert(Alert alert) {
    alert.set(false);
  }

  /**
   * Checks if any faults have occurred in the CANcoder device and activates the corresponding
   * alerts.
   *
   * @return True if at least one fault is detected; otherwise, false.
   */
  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            () -> LogError("UndervoltageAlert", encoder.getFault_Undervoltage().getValue()),
                underVoltAlert,
            () -> LogError("BasMagnetAlert", encoder.getFault_BadMagnet().getValue()),
                magnetOutOfRangeAlert,
            () -> LogError("HardwareAlert", encoder.getFault_Hardware().getValue()), HardwareAlert,
            () ->
                    LogError(
                        "UnlicensedFeatureInUseAlert",
                        encoder.getFault_UnlicensedFeatureInUse().getValue()),
                UnlicensedFeatureInUseAlert,
            () -> LogError("BootDuringEnableAlert", encoder.getFault_BootDuringEnable().getValue()),
                bootDuringEnableAlert);

    faultChecks.forEach(
        (faultCondition, alert) -> {
          if (faultCondition.getAsBoolean()) {
            foundFaults.add(alert);
          }
        });

    foundFaults.forEach(this::activateAlert);

    return !foundFaults.isEmpty();
  }

  /**
   * logs the state and returns the alert state.
   *
   * @param key which represents the advantage scope key that is being written too.
   * @param value from getting if the hardware has an error.
   * @return the logged state.
   */
  public boolean LogError(String key, boolean value) {
    Logger.recordOutput("Alerts/CanAndCoder/" + id + "/" + key, value);
    return value;
  }
}
