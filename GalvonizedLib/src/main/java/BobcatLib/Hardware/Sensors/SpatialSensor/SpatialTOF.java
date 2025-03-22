package BobcatLib.Hardware.Sensors.SpatialSensor;

import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import java.util.HashMap;
import java.util.List;

/**
 * The SpatialTOF class implements the SpatialIO interface. It is responsible for managing and
 * interacting with time-of-flight (TOF) sensors. The class handles updating inputs, configuring
 * sensors, and detecting objects in the environment.
 */
public class SpatialTOF implements SpatialIO {

  /** Sensors as identified by "field centric" side. */
  public List<RangeSensor> mRangeSensors;

  /**
   * Constructs a SpatialTOF object with the specified list of range sensors. The sensors are
   * configured after construction.
   *
   * @param sensors A list of {@link RangeSensor} objects representing the spatial sensors.
   */
  public SpatialTOF(List<RangeSensor> sensors) {
    this.mRangeSensors = sensors;
    configAllSensors();
  }

  /**
   * Updates the gyro inputs based on external sources. This method retrieves the distances from the
   * sensors and updates the alignment status.
   *
   * @param inputs The SpatialIOInputs object to update with the new sensor data.
   */
  public void updateInputs(SpatialIOInputs inputs) {
    HashMap<String, Double> distances = detectObjects();
    inputs.front_left_distance = distances.get("left");
    inputs.front_right_distance = distances.get("right");
    inputs.isAligned = isSquared(inputs.front_left_distance, inputs.front_right_distance, 15);
  }

  /**
   * Configures all sensors. This method can be used to apply any necessary settings or calibration
   * to the sensors.
   */
  public void configAllSensors() {}

  /**
   * Detects objects and calculates the distances from the left and right sensors.
   *
   * @return A {@link HashMap} with the distances from the left and right sensors in millimeters.
   */
  public HashMap<String, Double> detectObjects() {
    HashMap<String, Double> mDistances = new HashMap<String, Double>();

    mDistances.put("left", mRangeSensors.get(0).getRange());
    mDistances.put("right", mRangeSensors.get(1).getRange());

    return mDistances;
  }

  /**
   * Checks if two lengths are approximately equal within a given tolerance. This is useful for
   * determining if the system is aligned or squared.
   *
   * @param l1 The first length in mm.
   * @param l2 The second length in mm.
   * @param tolerance The acceptable difference between the two lengths in mm.
   * @return {@code true} if the absolute difference between l1 and l2 is within the tolerance,
   *     {@code false} otherwise.
   */
  public boolean isSquared(double l1, double l2, double tolerance) {
    return Math.abs(l1 - l2) <= tolerance;
  }

  /**
   * Retrieves the list of range sensors used in the Spatial system.
   *
   * @return A list of {@link RangeSensor} objects.
   */
  public List<RangeSensor> getRangeSensors() {
    return this.mRangeSensors;
  }
}
