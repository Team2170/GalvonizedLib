package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulates a gyro sensor for testing or development purposes. This class mimics the behavior of a
 * gyro sensor by maintaining simulated yaw, pitch, and roll values and updating them periodically.
 * It uses a timer to track the passage of time and simulate the changes in orientation over time.
 */
public class GyroSim implements GyroIO {

  /** Main timer to control movement estimations. */
  private Timer timer;

  /** The last time the timer was read, used to determine position changes. */
  private double lastTime;

  /** Heading of the robot (yaw angle). */
  private double yawAngle;

  /** Pitch angle of the robot. */
  private double pitchAngle;

  /** Roll angle of the robot. */
  private double rollAngle;

  private double accel;

  /** Initializes the GyroSim with a timer and sets up the initial angles. */
  public GyroSim() {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    configGyro();
  }

  /**
   * Configures the gyro simulation. Currently does nothing but can be extended for future
   * configuration.
   */
  public void configGyro() {}

  /**
   * Updates the gyro inputs based on the simulated gyro data. The simulated yaw, pitch, and roll
   * values are converted to {@link Rotation2d} objects and stored in the provided {@link
   * GyroIOInputs}.
   *
   * @param inputs The inputs to update with the simulated gyro data.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.rollPosition = Rotation2d.fromDegrees(getRoll());
    inputs.pitchPosition = Rotation2d.fromDegrees(getPitch());
    inputs.accel = getAccel();
  }

  /**
   * Sets the yaw value of the simulated gyro.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    this.yawAngle = yaw;
  }

  /**
   * Sets the pitch value of the simulated gyro.
   *
   * @param pitch The pitch value to set (in degrees).
   */
  public void setPitch(double pitch) {
    this.pitchAngle = pitch;
  }

  /**
   * Sets the roll value of the simulated gyro.
   *
   * @param roll The roll value to set (in degrees).
   */
  public void setRoll(double roll) {
    this.rollAngle = roll;
  }

  /**
   * Periodic update method for the Pigeon2 IMU sensor. Currently does nothing but can be extended
   * for specific periodic updates for Pigeon2.
   *
   * @param imu The Pigeon2 IMU instance.
   */
  public void periodic(Pigeon2 imu) {}

  /**
   * Periodic update method for the simulated gyro. This method is called periodically to update the
   * simulated gyro sensor's state.
   */
  public void periodic() {}

  /**
   * Gets the current yaw angle of the simulated gyro.
   *
   * @return The current yaw angle (in degrees).
   */
  private double getYaw() {
    return yawAngle;
  }

  /**
   * Gets the current pitch angle of the simulated gyro.
   *
   * @return The current pitch angle (in degrees).
   */
  private double getPitch() {
    return pitchAngle;
  }

  /**
   * Gets the current roll angle of the simulated gyro.
   *
   * @return The current roll angle (in degrees).
   */
  private double getRoll() {
    return rollAngle;
  }

  private double getAccel() {
    return accel;
  }

  /**
   * Gets the time difference between the last and current gyro update. This is used to simulate the
   * passage of time in the gyro's state.
   *
   * @return The time difference (in seconds) since the last update.
   */
  public double getTimeDiff() {
    return (timer.get() - lastTime);
  }

  public Rotation3d getGyroRates() {
    return new Rotation3d();
  }
}
