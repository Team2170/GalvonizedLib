package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

/**
 * The LaserCAN class represents a range sensor that uses the LaserCan hardware over a CAN
 * interface. It provides methods for configuring the sensor, retrieving measurements, and handling
 * faults.
 */
public class LaserCAN implements RangeSensor {

  public int id;
  public LaserCan tof;
  public DistanceMode mode;
  public double range;
  public Alert sensorAlert;
  public double sampleTime;

  /**
   * Constructs a new LaserCAN sensor instance with the specified parameters.
   *
   * @param id The ID of the range sensor.
   * @param mode The mode of operation for the sensor (e.g., distance mode).
   * @param sampleTime The time interval between sensor readings in seconds.
   */
  public LaserCAN(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      tof = new LaserCan(id);
      configRangeSensor();
    } catch (Exception e) {
      // Handle the exception and generate an alert
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occurred", level);
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
    LaserCan.Measurement measurement = tof.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      range = measurement.distance_mm;
    }
    return range;
  }

  /**
   * Configures the range sensor with default settings. This method applies the necessary
   * configuration to the LaserCAN sensor for basic usage.
   */
  public void configRangeSensor() {
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      tof.setRangingMode(mode.asLaserCAN());
      tof.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
      tof.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Configures the range sensor with a specific distance mode.
   *
   * @param m The desired distance mode (e.g., short, medium, or long).
   */
  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      tof.setRangingMode(m.asLaserCAN());
      tof.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
      tof.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
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
   * Retrieves the optimal distance mode based on the current range reading. The optimal mode is
   * chosen based on the distance measured by the sensor.
   *
   * <p>RangingMode.Short: if distance less than 1250 mm<br>
   * RangingMode.Medium: if distance between 1250 mm and 2250 mm<br>
   * RangingMode.Long: if distance greater than or equal to 2250 mm
   *
   * @return The optimal distance mode based on the current range.
   */
  public DistanceMode getOptimalMode() {
    double distance = getRange();
    RangingMode mode =
        (distance < 1000)
            ? RangingMode.SHORT
            : (distance < 1000) ? RangingMode.SHORT : RangingMode.LONG;
    DistanceMode distanceMode = new DistanceMode();
    return distanceMode.fromLaserCAN(mode);
  }

  /**
   * Updates the sensor's state based on directional translation. This method can be used to adjust
   * the sensor state based on movement or translation.
   *
   * @param translation The translation value used to adjust the sensor state.
   */
  public void updateFromDirectional(double translation) {}
}
