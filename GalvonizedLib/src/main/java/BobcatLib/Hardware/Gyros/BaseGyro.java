package BobcatLib.Hardware.Gyros;

import BobcatLib.Hardware.Gyros.GyroIO.GyroIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The BaseGyro class provides an abstraction for interacting with a gyro sensor. It communicates
 * with the underlying GyroIO interface to retrieve and set orientation data (yaw, pitch, and roll)
 * and periodically updates sensor inputs.
 */
public class BaseGyro {

  /** The GyroIO interface used to interact with the gyro sensor. */
  private final GyroIO io;

  /** The inputs object holding the current gyro sensor values. */
  private final GyroIOInputs inputs = new GyroIOInputs();

  /** The name of the gyro sensor (used for identification). */
  private final String name;

  /**
   * Constructor to initialize a BaseGyro instance.
   *
   * @param name The name of the gyro sensor for identification.
   * @param io The GyroIO interface used to interact with the gyro hardware.
   */
  public BaseGyro(String name, GyroIO io) {
    this.io = io;
    this.name = name;
  }

  /**
   * Periodically updates the sensor inputs from the GyroIO interface. This should be called
   * regularly to refresh the gyro's data.
   */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The desired yaw value to set.
   */
  public void setYaw(double yaw) {
    io.setYaw(yaw);
  }

  /**
   * Sets the roll value of the gyro sensor.
   *
   * @param roll The desired roll value to set.
   */
  public void setRoll(double roll) {
    io.setRoll(roll);
  }

  /**
   * Sets the pitch value of the gyro sensor.
   *
   * @param pitch The desired pitch value to set.
   */
  public void setPitch(double pitch) {
    io.setPitch(pitch);
  }

  /**
   * Gets the current yaw position of the gyro as a Rotation2d object.
   *
   * @return The yaw position of the gyro.
   */
  public Rotation2d getYaw() {
    return inputs.yawPosition;
  }

  /**
   * Gets the current pitch position of the gyro as a Rotation2d object.
   *
   * @return The pitch position of the gyro.
   */
  public Rotation2d getPitch() {
    return inputs.pitchPosition;
  }

  /**
   * Gets the current roll position of the gyro as a Rotation2d object.
   *
   * @return The roll position of the gyro.
   */
  public Rotation2d getRoll() {
    return inputs.rollPosition;
  }

  /**
   * Retrieves the time difference between the last and current sensor update. This can be useful
   * for determining how much time has passed between updates.
   *
   * @return The time difference between the last and current update in seconds.
   */
  public double getTimeDiff() {
    return io.getTimeDiff();
  }

  public double getAccel() {
    return inputs.accel;
  }

  public Rotation3d getRotationRate() {
    return io.getGyroRates();
  }
}
