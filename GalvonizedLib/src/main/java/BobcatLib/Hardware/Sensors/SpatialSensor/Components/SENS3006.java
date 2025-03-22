package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * The SENS3006 class represents a Time-of-Flight (ToF) sensor that measures the distance to an
 * object in front of it. It implements the RangeSensor interface and provides specific
 * functionality to configure and retrieve distance data from the SENS3006 sensor.
 */
public class SENS3006 implements RangeSensor {

  public int id;
  public TimeOfFlight tof;
  public final double sampleTime;
  public DistanceMode mode;
  public double range;
  public Alert sensorAlert;

  /**
   * Constructs a new SENS3006 instance and initializes the sensor with the given parameters. The
   * sensor is configured with the default settings for the specified distance mode and sample time.
   *
   * @param id The unique identifier for the sensor.
   * @param mode The distance mode (e.g., short, medium, or long range).
   * @param sampleTime The time interval between sensor measurements.
   */
  public SENS3006(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      tof = new TimeOfFlight(id);
      configRangeSensor();
    } catch (Exception e) {
      // Handle exception and create an alert if the sensor hardware fails
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  /**
   * Retrieves the range (distance) in front of the sensor. This method returns the distance
   * measured by the sensor in millimeters.
   *
   * @return The range in millimeters.
   */
  public double getRange() {
    range = 0;
    range = tof.getRange();
    return range;
  }

  /**
   * Configures the sensor with the default ranging mode based on the specified distance mode and
   * sample time. This sets up the sensor's parameters to measure distances.
   */
  public void configRangeSensor() {
    tof.setRangingMode(mode.asPWF(), sampleTime);
  }

  /**
   * Configures the sensor with the specified distance mode and sample time.
   *
   * @param m The new distance mode to be applied to the sensor (e.g., short, medium, long).
   */
  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
    tof.setRangingMode(mode.asPWF(), sampleTime);
  }

  /**
   * Gets the current distance mode of the sensor.
   *
   * @return The current distance mode used by the sensor.
   */
  public DistanceMode getMode() {
    return mode;
  }

  /**
   * Determines the optimal distance mode for the sensor based on the current range. The method
   * assigns a ranging mode (Short, Medium, or Long) based on the current measured distance to
   * provide the best sensor performance.
   *
   * @return The optimal distance mode for the current sensor reading.
   */
  public DistanceMode getOptimalMode() {
    double distance = getRange();
    RangingMode mode =
        (distance < 1250)
            ? RangingMode.Short
            : (distance < 2250) ? RangingMode.Medium : RangingMode.Long;
    DistanceMode distanceMode = new DistanceMode();
    return distanceMode.fromPWF(mode);
  }

  /**
   * Updates the sensor's state based on a directional translation input. This method may be used
   * for processing input from the robot's movement or other system changes.
   *
   * @param translation The translation value that represents the directional change.
   */
  public void updateFromDirectional(double translation) {}
}
