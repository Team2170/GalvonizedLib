package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages and monitors faults for the Pigeon2 IMU device. This class provides alerts for various
 * fault conditions that can occur during the operation of the Pigeon2 IMU.
 */
public class Pigeon2Faults implements FaultsWrapper {

  /** The unique identifier for the Pigeon2 device. */
  public int id;

  /** The Pigeon2 IMU instance being monitored. */
  public Pigeon2 imu;

  // Static alert definitions for various fault types
  public static Alert BootDuringEnableAlert;
  public static Alert SaturatedAccelerometerAlert;
  public static Alert SaturatedMagnetometerAlert;
  public static Alert SaturatedGyroScopeAlert;
  public static Alert LoopTimeSlowFaultAlert;
  public static Alert DataAcquiredLateFaultAlert;
  public static Alert BootIntoMotionFaultAlert;
  public static Alert BootupMagnetometerFaultAlert;
  public static Alert BootupGyroscopeFaultAlert;
  public static Alert UndervoltageFaultAlert;
  public static Alert HardwareAlert;

  /**
   * Constructs a new Pigeon2Faults instance with a given Pigeon2 IMU and ID.
   *
   * @param imu The Pigeon2 IMU to monitor.
   * @param id The unique identifier for the Pigeon2 device.
   */
  public Pigeon2Faults(Pigeon2 imu, int id) {
    this.id = id;
    this.imu = imu;
    AlertType level = AlertType.INFO;

    BootDuringEnableAlert =
        new Alert("Gyro", "Pigeon2 " + id + " boot while detecting the enable signal", level);
    SaturatedAccelerometerAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Accelerometer values are saturated", level);
    SaturatedMagnetometerAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Magnetometer values are saturated", level);
    SaturatedGyroScopeAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Gyroscope values are saturated", level);
    LoopTimeSlowFaultAlert =
        new Alert(
            "Gyro", "Pigeon2 " + id + " Motion stack loop time was slower than expected.", level);
    DataAcquiredLateFaultAlert =
        new Alert(
            "Gyro",
            "Pigeon2 " + id + " Motion stack data acquisition was slower than expected",
            level);
    BootIntoMotionFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " motion Detected during bootup.", level);
    BootupMagnetometerFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Bootup checks failed: Magnetometer", level);
    BootupGyroscopeFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Bootup checks failed: Gyroscope", level);
    UndervoltageFaultAlert =
        new Alert(
            "Gyro", "Pigeon2 " + id + " supply voltage dropped to near brownout levels", level);
    HardwareAlert = new Alert("Gyro", "Pigeon2 " + id + " Hardware fault occurred", level);
  }

  /**
   * Activates an alert, marking it as triggered.
   *
   * @param alert The alert to activate.
   */
  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("Pigeon2 " + id);
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
   * Checks if a specific fault condition is true and triggers an alert if so.
   *
   * @param faultTest The fault condition to evaluate.
   * @param alert The alert to activate if the fault condition is true.
   * @param alertType The severity level for the alert.
   * @return {@code true} if the fault condition is true; otherwise, {@code false}.
   */
  public boolean detect_fault(boolean faultTest, Alert alert, AlertType alertType) {
    boolean fault = false;
    if (faultTest) {
      fault = true;
    }
    return fault;
  }

  /**
   * Checks if any fault has occurred for the Pigeon2 IMU. If faults are detected, corresponding
   * alerts are activated.
   *
   * @return {@code true} if at least one fault is detected; otherwise, {@code false}.
   */
  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            () -> LogError("BootDuringEnableAlert", imu.getFault_BootDuringEnable().getValue()),
            BootDuringEnableAlert,
            () ->
                LogError(
                    "SaturatedAccelerometerAlert",
                    imu.getFault_SaturatedAccelerometer().getValue()),
            SaturatedAccelerometerAlert,
            () ->
                LogError(
                    "SaturatedMagnetometerAlert", imu.getFault_SaturatedMagnetometer().getValue()),
            SaturatedMagnetometerAlert,
            () -> LogError("SaturatedGyroScopeAlert", imu.getFault_SaturatedGyroscope().getValue()),
            SaturatedGyroScopeAlert,
            () -> LogError("LoopTimeSlowFaultAlert", imu.getFault_LoopTimeSlow().getValue()),
            LoopTimeSlowFaultAlert,
            () ->
                LogError("DataAcquiredLateFaultAlert", imu.getFault_DataAcquiredLate().getValue()),
            DataAcquiredLateFaultAlert,
            () -> LogError("BootIntoMotionFaultAlert", imu.getFault_BootIntoMotion().getValue()),
            BootIntoMotionFaultAlert,
            () ->
                LogError(
                    "BootupMagnetometerFaultAlert", imu.getFault_BootupMagnetometer().getValue()),
            BootupMagnetometerFaultAlert,
            () -> LogError("UndervoltageFaultAlert", imu.getFault_Undervoltage().getValue()),
            UndervoltageFaultAlert,
            () -> LogError("HardwareAlert", imu.getFault_Hardware().getValue()),
            HardwareAlert);

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
    Logger.recordOutput("Alerts/Pigeon2/" + id + "/" + key, value);
    return value;
  }
}
