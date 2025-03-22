package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface PoseLib {
  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public default Pose2d getEstimatedPosition() {
    return new Pose2d();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters The position on the field that your robot is at.
   */
  public default void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    return;
  }

  public default Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    return new Pose2d();
  }

  public default Pose2d updateWithTimeAndDevs(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    return new Pose2d();
  }
}
