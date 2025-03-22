package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Represents an interface for interacting with a gyro sensor. This interface defines methods for
 * updating sensor inputs, setting and retrieving gyro data, and performing periodic updates.
 */
public interface GyroIO {

  /**
   * Represents the inputs for the gyro sensor. This class stores data about the gyro's connection
   * status, orientation (yaw, pitch, roll), and fault status.
   */
  public class GyroIOInputs {

    /** Indicates if the gyro is connected. */
    public boolean connected = false;

    /** The current yaw position of the gyro. */
    public Rotation2d yawPosition = new Rotation2d();

    /** The current pitch position of the gyro. */
    public Rotation2d pitchPosition = new Rotation2d();

    /** The current roll position of the gyro. */
    public Rotation2d rollPosition = new Rotation2d();

    public double accel = 0;

    /** Indicates if the gyro has a fault. */
    public boolean faulted = false;
  }

  /**
   * Updates the gyro inputs based on external sources. This method is called to refresh the gyro's
   * internal data.
   *
   * @param inputs The object that stores the updated gyro inputs.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Gets the time difference between the last and current sensor update. This can be used to track
   * the time between periodic updates.
   *
   * @return The time difference between the last and current update in seconds.
   */
  public default double getTimeDiff() {
    return 1.0;
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public default void setYaw(double yaw) {}

  /**
   * Sets the pitch value of the gyro sensor.
   *
   * @param pitch The pitch value to set (in degrees).
   */
  public default void setPitch(double pitch) {}

  /**
   * Sets the roll value of the gyro sensor.
   *
   * @param roll The roll value to set (in degrees).
   */
  public default void setRoll(double roll) {}

  /**
   * Periodically updates the gyro sensor's data for a Pigeon2 device. This method should be called
   * regularly to update the gyro's state.
   *
   * @param imu The Pigeon2 IMU sensor instance to update.
   */
  public default void periodic(Pigeon2 imu) {}

  /**
   * Periodically updates the gyro sensor's data. This method should be called regularly to update
   * the gyro's state.
   */
  public default void periodic() {}

  public default Rotation3d getGyroRates() {
    return new Rotation3d(0, 0, 0);
  }
}
