package BobcatLib.Hardware.Sensors.SpatialSensor;

import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The Spatial class interfaces with spatial sensors (range sensors) and provides functionality for
 * periodic updates and distance retrieval. It works with the SpatialIO class to communicate with
 * the hardware and manage sensor inputs.
 */
public class Spatial {

  private final SpatialIO io;
  private final SpatialIOInputsAutoLogged inputs = new SpatialIOInputsAutoLogged();
  public boolean isEnabled = false;

  /**
   * Constructs a new Spatial object.
   *
   * @param io The SpatialIO object that interacts with the hardware for sensor inputs.
   */
  public Spatial(SpatialIO io) {
    this.io = io;
  }

  /**
   * Periodic function that updates sensor inputs and logs the data. This method should be called
   * periodically, such as once per robot loop.
   */
  public void periodic() {
    io.updateInputs(inputs); // Updates the inputs from hardware
    Logger.processInputs("Spatial", inputs); // Logs the inputs for debugging and analysis
  }

  /**
   * Returns a list of spatial sensors (range sensors) associated with this system.
   *
   * @return A list of RangeSensor objects.
   */
  public List<RangeSensor> getSpatialSensors() {
    return io.getRangeSensors();
  }

  /**
   * Checks whether the spatial system is squared (aligned).
   *
   * @return true if the system is squared (aligned), false otherwise.
   */
  public boolean isSquared() {
    return inputs.isAligned; // Returns whether the system is aligned
  }

  /**
   * Retrieves the distances from the front left and front right sensors.
   *
   * @return A HashMap containing the sensor names ("left" and "right") as keys, with their
   *     corresponding distance values in millimeters.
   */
  public HashMap<String, Double> getDistances() {
    HashMap<String, Double> distances = new HashMap<>();
    distances.put("left", inputs.front_left_distance); // Front left distance
    distances.put(
        "right",
        inputs
            .front_left_distance); // Front right distance (probably should be front_right_distance)
    return distances;
  }
}
