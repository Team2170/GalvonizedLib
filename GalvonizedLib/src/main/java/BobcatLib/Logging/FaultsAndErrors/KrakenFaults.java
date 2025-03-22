package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages fault detection and alerting for TalonFX motor controllers. Provides methods for
 * creating, activating, and deactivating alerts for various fault conditions.
 */
public class KrakenFaults implements FaultsWrapper {

  /** Enumeration of possible faults that can occur in the TalonFX motor controller. */
  public enum faults {
    hardware,
    procTemp,
    deviceTemp,
    underVoltage,
    bootDuringEnable,
    unlicensedFeatureInUse,
    unstableSupplyV
  }

  /** Alert for hardware faults. */
  public static Alert hardwareAlert;

  /** Alert for processor temperature exceeding safe limits. */
  public static Alert procTempAlert;

  /** Alert for device temperature exceeding safe limits. */
  public static Alert deviceTempAlert;

  /** Alert for undervoltage conditions near brownout levels. */
  public static Alert underVoltageAlert;

  /** Alert for boot events during enable signal detection. */
  public static Alert bootDuringEnableAlert;

  /** Alert for using unlicensed features. */
  public static Alert unlicensedFeatureInUseAlert;

  /** Alert for unstable supply voltage. */
  public static Alert unstableSupplyVAlert;

  /** Unique identifier for the TalonFX motor controller. */
  public int id;

  /** TalonFX motor controller instance. */
  public TalonFX motor;

  /**
   * Constructs a new instance of `TalonFXFaults` for a given motor controller.
   *
   * @param motor The TalonFX motor controller instance.
   * @param id The unique identifier of the TalonFX motor controller.
   */
  public KrakenFaults(TalonFX motor, int id) {
    this.id = id;
    this.motor = motor;
    AlertType level = AlertType.INFO;
    hardwareAlert = new Alert("Motors", "TalonFX " + id + " hardware fault occurred", level);
    procTempAlert =
        new Alert("Motors", "TalonFX " + id + " Processor temperature exceeded limit", level);
    deviceTempAlert =
        new Alert("Motors", "TalonFX " + id + " Device temperature exceeded limit", level);
    underVoltageAlert =
        new Alert(
            "Motors",
            "TalonFX " + id + " Device supply voltage dropped to near brownout levels",
            level);
    bootDuringEnableAlert =
        new Alert(
            "Motors", "TalonFX " + id + " Device boot while detecting the enable signal", level);
    unlicensedFeatureInUseAlert =
        new Alert(
            "Motors",
            "TalonFX "
                + id
                + " An unlicensed feature is in use, device may not behave as expected.",
            level);
    unstableSupplyVAlert =
        new Alert("Motors", "TalonFX " + id + " unstable voltage detected.", level);
  }

  /**
   * Activates an alert, marking it as triggered and logs the alert with the associated TalonFX ID.
   *
   * @param alert The alert to activate.
   */
  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("TalonFX " + id);
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
   * Detects a fault condition and returns whether the fault was detected.
   *
   * @param faultTest A boolean indicating if the fault condition is true.
   * @param alert The alert to trigger if the fault is detected.
   * @param alertType The severity level of the fault.
   * @return True if the fault condition is true; otherwise, false.
   */
  public boolean detect_fault(boolean faultTest, Alert alert, AlertType alertType) {
    return faultTest;
  }

  /**
   * Checks for any faults in the TalonFX motor controller and activates corresponding alerts.
   *
   * @return True if any fault conditions are detected; otherwise, false.
   */
  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            motor.getFault_ProcTemp()::getValue, procTempAlert,
            motor.getFault_DeviceTemp()::getValue, deviceTempAlert,
            motor.getFault_Undervoltage()::getValue, underVoltageAlert,
            motor.getFault_BootDuringEnable()::getValue, bootDuringEnableAlert,
            motor.getFault_UnlicensedFeatureInUse()::getValue, unlicensedFeatureInUseAlert,
            motor.getFault_UnstableSupplyV()::getValue, unstableSupplyVAlert,
            motor.getFault_Hardware()::getValue, hardwareAlert);

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
    Logger.recordOutput("Alerts/Kraken/" + id + "/" + key, value);
    return value;
  }
}
