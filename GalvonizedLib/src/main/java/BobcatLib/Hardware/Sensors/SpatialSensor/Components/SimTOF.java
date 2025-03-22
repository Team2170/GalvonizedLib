package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimDouble;

/**
 * The SimTOF class simulates a Time-of-Flight (ToF) sensor for use in simulation environments. It
 * provides methods for getting the range and configuring the sensor's distance mode.
 */
public class SimTOF implements RangeSensor {

  public int id;
  public DistanceMode mode;
  public double range = 100;
  public Alert sensorAlert;
  public double sampleTime;
  public SimDevice simDevice;

  // Sim Stuff
  public SimDouble simRange;

  /**
   * Constructs a new SimTOF instance for simulation. This method initializes the simulated device
   * and its range.
   *
   * @param id The unique identifier for the sensor.
   * @param mode The distance mode (e.g., short, medium, or long range).
   * @param sampleTime The time interval between sensor measurements.
   */
  public SimTOF(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    simDevice = new SimDevice(SimDeviceJNI.createSimDevice("SimTOF [" + id + "]"));
    simRange = simDevice.createDouble("range", Direction.kBidir, 0.00);
    try {
      configRangeSensor();
    } catch (Exception e) {
      // Handle exception and create an alert if the sensor hardware fails
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occurred", level);
      sensorAlert.set(true);
    }
  }

  /**
   * Retrieves the range (distance) in front of the sensor. This method returns the simulated
   * distance in millimeters.
   *
   * @return The simulated range in millimeters.
   */
  public double getRange() {
    range = simRange.get();
    return range;
  }

  /**
   * Configures the sensor with default settings. Since this is a simulation, no specific
   * configuration is needed, but this method can be extended if necessary.
   */
  public void configRangeSensor() {}

  /**
   * Configures the sensor with the specified distance mode. This method updates the mode of the
   * simulated sensor.
   *
   * @param m The new distance mode to be applied to the sensor (e.g., short, medium, long).
   */
  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
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
   * Determines the optimal distance mode for the sensor based on the current range. Since this is a
   * simulation, the method returns a fixed value (SHORT).
   *
   * @return The optimal distance mode for the current sensor reading.
   */
  public DistanceMode getOptimalMode() {
    DistanceMode distanceMode = new DistanceMode(DistanceMode.modes.SHORT);
    return distanceMode;
  }
}
