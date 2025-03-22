package BobcatLib.Hardware.Gyros;

import BobcatLib.Hardware.Gyros.Parser.GyroDeviceJson;
import BobcatLib.Hardware.Gyros.Parser.GyroJson;
import BobcatLib.Logging.FaultsAndErrors.BoronFaults;
import BobcatLib.Logging.FaultsAndErrors.FaultsWrapper;
import BobcatLib.Utilities.CANDeviceDetails;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

/**
 * Represents a Pigeon2 gyro sensor, providing methods to interact with the gyro hardware. It can
 * load its configuration from a file, apply settings, and handle faults.
 */
public class BoronGyro implements GyroIO {
  private FaultsWrapper gyroFaults;
  private Canandgyro gyro;

  // Subscribed to the configuration changes
  public StringSubscriber configSubscriber;
  public String lastConfig = "{}";
  public GyroJson jsonGyro = new GyroJson();

  /**
   * Constructs a Pigeon2Gyro instance with the specified CAN device details and enables the gyro.
   *
   * @param details The details of the CAN device, including the device number and CAN bus.
   * @param enableGyro Whether to enable the gyro functionality.
   */
  public BoronGyro(CANDeviceDetails details, boolean enableGyro) {
    jsonGyro.imu = new GyroDeviceJson(details.getDeviceNumber(), details.getBus());
    jsonGyro.enable = enableGyro;
    configGyro();
  }

  /**
   * Default constructor that loads the gyro configuration from a file and applies the
   * configuration.
   */
  public BoronGyro() {
    loadConfigurationFromFile();
    configGyro();
  }

  /**
   * Configures the gyro sensor by applying a Pigeon2 configuration, resetting it, and setting the
   * yaw to zero.
   */
  public void configGyro() {
    gyro = new Canandgyro(jsonGyro.imu.getId());
    gyro.resetFactoryDefaults(0.35);
    gyro.setYaw(0);
    gyroFaults = new BoronFaults(gyro, gyro.getAddress().getDeviceId());
  }

  /**
   * Loads the gyro configuration from a JSON file stored in the robot's deploy directory.
   *
   * @return The loaded GyroJson object containing the gyro configuration.
   */
  public GyroJson loadConfigurationFromFile() {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, "gyro.json").exists();
    File gyroFile = new File(directory, "gyro.json");
    assert gyroFile.exists();

    try {
      jsonGyro = new ObjectMapper().readValue(gyroFile, GyroJson.class);
    } catch (IOException e) {
      jsonGyro.imu = new GyroDeviceJson(35, "");
    }
    return jsonGyro;
  }

  /**
   * Periodic method to be called during each cycle, can be extended to implement periodic updates.
   */
  public void periodic() {}

  /**
   * Updates the gyro inputs based on the current gyro data.
   *
   * @param inputs The inputs to be updated with the current yaw and fault status.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.faulted = gyroFaults.hasFaultOccured();
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set, in degrees.
   */
  public void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  /**
   * Gets the current yaw value of the gyro sensor.
   *
   * @return The current yaw value, in degrees.
   */
  private double getYaw() {
    Rotation2d internal = Rotation2d.fromRotations(gyro.getYaw());
    return internal.getDegrees();
  }

  /**
   * Sets the roll value of the gyro sensor. Currently, this method is a placeholder and does not
   * modify the roll value.
   *
   * @param roll The roll value to set, in degrees.
   */
  public void setRoll(double roll) {}

  /**
   * Gets the current roll value of the gyro sensor.
   *
   * @return The current roll value, in degrees.
   */
  private double getRoll() {
    Rotation2d internal = Rotation2d.fromRotations(gyro.getRoll());
    return internal.getDegrees();
  }

  /**
   * Sets the pitch value of the gyro sensor. Currently, this method is a placeholder and does not
   * modify the pitch value.
   *
   * @param pitch The pitch value to set, in degrees.
   */
  public void setPitch(double pitch) {}

  /**
   * Gets the current pitch value of the gyro sensor.
   *
   * @return The current pitch value, in degrees.
   */
  private double getPitch() {
    Rotation2d internal = Rotation2d.fromRotations(gyro.getPitch());
    return internal.getDegrees();
  }

  /**
   * Gets the time difference since the last update.
   *
   * @return The time difference, currently returning a fixed value of 1.0 second.
   */
  public double getTimeDiff() {
    return 1.0;
  }

  public Rotation3d getGyroRates() {
    return new Rotation3d();
  }
}
