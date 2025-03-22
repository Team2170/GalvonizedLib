package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Manages and monitors faults for the Boron Canandgyro device. This class provides alerts for
 * various fault conditions that can occur during the operation of the Boron Canandgyro.
 */
public class BoronFaults implements FaultsWrapper {

  /** The unique identifier for the Boron CanAndGyro device. */
  public int id;

  /** The Boron CanAndGyro instance being monitored. */
  public Canandgyro imu;

  // Static alert definitions for various fault types
  public static Alert PowerCycleAlert;
  public static Alert CanIdConflictAlert;
  public static Alert CanGeneralErrorAlert;
  public static Alert OutOfTemperatureRangeAlert;
  public static Alert HardwareFaultAlert;
  public static Alert CalibratingAlert;
  public static Alert AngularVelcoitySaturationAlert;
  public static Alert AccelerationSaturationAlert;

  /**
   * Constructs a new BoronFaults instance with a given Boron CanAndGyro and ID.
   *
   * @param imu The Boron CanAndGyro to monitor.
   * @param id The unique identifier for the Boron CanAndGyro device.
   */
  public BoronFaults(Canandgyro imu, int id) {
    this.id = id;
    this.imu = imu;
    AlertType level = AlertType.INFO;

    PowerCycleAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " The Power Cycle fault is set to true when the gyro powers on. By clearing sticky faults on initialization, this flag can be used to determine if the encoder has rebooted.",
            level);
    CanIdConflictAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " The CAN ID Conflict flag is set to true when the device detects another device with the same CAN ID. While available in the API, you should check the physical device for the LED flashing blue (which indicates a detected CAN ID conflict). Alchemist should be used to configure the different devices to avoid CAN ID conflicts.",
            level);
    CanGeneralErrorAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " The CAN General Error flag is set to true if the encoder detects a problem with its ability to recieve CAN packets, or other issues with the CAN bus. Generally, this indicates an issue with the CAN wiring.",
            level);
    OutOfTemperatureRangeAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " The Out Of Temperature Range flag is set to true if the encoder detects its temperature is outside the safe operating range.",
            level);
    HardwareFaultAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " A Hardware Fault indicates the device has detected a problem during operation. For the Boron, this generally will indicate that the device is unable to communicate with the on board magnetic field sensors.",
            level);
    CalibratingAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " The Calibrating flag is set to true if the device is currently calibrating.",
            level);
    AngularVelcoitySaturationAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " This fault is flagged if the rotation of the gyro exceeded the measurable rotation rate of the sensor.",
            level);
    AccelerationSaturationAlert =
        new Alert(
            "Gyro",
            "Pigeon2 "
                + id
                + " This fault is flagged if the acceleration experienced exceeds the measurable acceleration of the sensor.",
            level);
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
   * Checks if any fault has occurred for the Boron Canandgyro. If faults are detected,
   * corresponding alerts are activated.
   *
   * @return {@code true} if at least one fault is detected; otherwise, {@code false}.
   */
  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            () -> imu.getStickyFaults().powerCycle(),
            PowerCycleAlert,
            () -> imu.getStickyFaults().canIDConflict(),
            CanIdConflictAlert,
            () -> imu.getStickyFaults().canGeneralError(),
            CanGeneralErrorAlert,
            () -> imu.getStickyFaults().outOfTemperatureRange(),
            OutOfTemperatureRangeAlert,
            () -> imu.getStickyFaults().hardwareFault(),
            HardwareFaultAlert,
            () -> imu.getStickyFaults().calibrating(),
            CalibratingAlert,
            () -> imu.getStickyFaults().angularVelocitySaturation(),
            AngularVelcoitySaturationAlert,
            () -> imu.getStickyFaults().accelerationSaturation(),
            AccelerationSaturationAlert);

    faultChecks.forEach(
        (faultCondition, alert) -> {
          if (faultCondition.getAsBoolean()) {
            foundFaults.add(alert);
          }
        });

    foundFaults.forEach(this::activateAlert);

    return !foundFaults.isEmpty();
  }
}
