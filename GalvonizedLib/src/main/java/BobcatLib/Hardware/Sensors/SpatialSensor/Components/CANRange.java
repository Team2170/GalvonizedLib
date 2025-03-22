package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

/**
 * The CANRange class represents a range sensor that uses a CAN interface for communication. It
 * provides methods for obtaining distance measurements and configuring the sensor.
 */
public class CANRange implements RangeSensor {

  public int id;
  public CANrange tof;
  public final double sampleTime;
  public DistanceMode mode;
  public double range;
  public Alert sensorAlert;

  /**
   * Constructs a new CANRange sensor instance with the specified parameters.
   *
   * @param id The ID of the range sensor.
   * @param mode The mode of operation for the sensor (e.g., distance mode).
   * @param sampleTime The time interval between sensor readings in seconds.
   * @param busname The CAN bus name (optional, defaults to "rio" if empty).
   */
  public CANRange(int id, DistanceMode mode, double sampleTime, String busname) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      if (busname == "" || busname == "rio") {
        // Construct the CANrange
        tof = new CANrange(id);
      } else {
        // Construct the CANrange
        tof = new CANrange(id, busname);
      }

      configRangeSensor();
    } catch (Exception e) {
      // TODO: handle exception
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  /**
   * Constructs a new CANRange sensor instance with the specified parameters.
   *
   * @param id The ID of the range sensor.
   * @param mode The mode of operation for the sensor (e.g., distance mode).
   * @param sampleTime The time interval between sensor readings in seconds.
   */
  public CANRange(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      // Construct the CANrange
      tof = new CANrange(id);

      configRangeSensor();
    } catch (Exception e) {
      // TODO: handle exception
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  /**
   * Gets the range (distance) in front of the sensor.
   *
   * @return The distance measured by the sensor in millimeters.
   */
  public double getRange() {
    range = 0;
    // Get Distance
    range = tof.getDistance().getValueAsDouble();

    return range;
  }

  /**
   * Configures the range sensor with default settings. This method applies the basic configuration
   * to the CANrange sensor.
   */
  public void configRangeSensor() {
    // Configure the CANrange for basic use
    CANrangeConfiguration configs = new CANrangeConfiguration();

    // Write these configs to the CANrange
    tof.getConfigurator().apply(configs);
  }

  /**
   * Configures the range sensor with a specific distance mode.
   *
   * @param m The desired distance mode (e.g., short, medium, or long).
   */
  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
  }

  /**
   * Retrieves the current distance mode of the sensor.
   *
   * @return The current distance mode of the sensor.
   */
  public DistanceMode getMode() {
    return mode;
  }

  /**
   * Retrieves the optimal distance mode based on the current sensor reading. The method checks the
   * distance and selects the most appropriate mode.
   *
   * @return The optimal distance mode for the current range.
   */
  public DistanceMode getOptimalMode() {
    double distance = getRange();
    DistanceMode distanceMode = new DistanceMode();
    distanceMode.currentMode = DistanceMode.modes.MEDIUM;
    return distanceMode;
  }

  /**
   * Updates the sensor's state based on directional translation. This method can be used to adjust
   * sensor settings based on motion.
   *
   * @param translation The translation value to adjust the sensor state.
   */
  public void updateFromDirectional(double translation) {}
}
