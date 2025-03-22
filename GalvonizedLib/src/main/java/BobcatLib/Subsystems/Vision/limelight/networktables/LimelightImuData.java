package BobcatLib.Subsystems.Vision.limelight.networktables;

import BobcatLib.Subsystems.Vision.limelight.Limelight;
import edu.wpi.first.networktables.NetworkTable;

public class LimelightImuData {
  /**
   * External : 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2 localization.
   * The internal IMU is ignored entirely. Both : 1 - Use external IMU yaw submitted via
   * SetRobotOrientation(), and configure the LL4 internal IMU’s fused yaw to match the submitted
   * yaw value. Internal : 2 - Use internal IMU for MT2 localization. External imu data is ignored
   * entirely
   */
  public enum imuMode {
    External,
    Both,
    Internal
  }

  /** {@link NetworkTable} for the {@link Limelight} */
  private NetworkTable limelightTable;
  /** {@link Limelight} to fetch data for. */
  private Limelight limelight;

  public double robotYaw = 0.0;
  public double Roll = 0.0;
  public double Pitch = 0.0;
  public double Yaw = 0.0;
  public double gyroX = 0.0;
  public double gyroY = 0.0;
  public double gyroZ = 0.0;
  public double accelX = 0.0;
  public double accelY = 0.0;
  public double accelZ = 0.0;

  public LimelightImuData(Limelight camera) {
    this.limelight = camera;
  }

  public LimelightImuData(Limelight camera, double[] imuData) {
    this.limelight = camera;
    if (imuData != null && imuData.length >= 10) {
      this.robotYaw = imuData[0];
      this.Roll = imuData[1];
      this.Pitch = imuData[2];
      this.Yaw = imuData[3];
      this.gyroX = imuData[4];
      this.gyroY = imuData[5];
      this.gyroZ = imuData[6];
      this.accelX = imuData[7];
      this.accelY = imuData[8];
      this.accelZ = imuData[9];
    }
  }

  /**
   * Gets the current IMU data from NetworkTables. IMU data is formatted as [robotYaw, Roll, Pitch,
   * Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ]. Returns all zeros if data is invalid or
   * unavailable.
   *
   * @return {@link LimelightImuData} object containing all current IMU data
   */
  public LimelightImuData getIMUData() {
    double[] imuData = LimelightUtils.getLimelightNTDoubleArray(limelight.limelightName, "imu");
    if (imuData == null || imuData.length < 10) {
      return new LimelightImuData(limelight); // Returns object with all zeros
    }
    return new LimelightImuData(limelight, imuData);
  }
}
