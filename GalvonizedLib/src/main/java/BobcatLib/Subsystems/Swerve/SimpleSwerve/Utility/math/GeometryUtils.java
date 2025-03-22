package BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Provides utility methods for working with geometry in Swerve drive systems. */
public class GeometryUtils {
  /** A small constant value for numerical stability. */
  private static final double kEps = 1E-9;

  /**
   * Calculates the exponential of a twist, yielding a new pose.
   *
   * @param delta The twist to compute the exponential of.
   * @return The resulting pose after applying the twist.
   */
  public static Pose2d exp(final Twist2d delta) {
    double sinTheta = Math.sin(delta.dtheta);
    double cosTheta = Math.cos(delta.dtheta);
    double s = calculateS(delta.dtheta, sinTheta);
    double c = calculateC(delta.dtheta, cosTheta);
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cosTheta, sinTheta));
  }

  /**
   * Calculates the value of 's' used in the exponential calculation.
   *
   * @param dtheta The delta theta value.
   * @param sinTheta The sine of delta theta.
   * @return The 's' value.
   */
  private static double calculateS(double dtheta, double sinTheta) {
    return Math.abs(dtheta) < kEps ? 1.0 - 1.0 / 6.0 * dtheta * dtheta : sinTheta / dtheta;
  }

  /**
   * Calculates the value of 'c' used in the exponential calculation.
   *
   * @param dtheta The delta theta value.
   * @param cosTheta The cosine of delta theta.
   * @return The 'c' value.
   */
  private static double calculateC(double dtheta, double cosTheta) {
    return Math.abs(dtheta) < kEps ? .5 * dtheta : (1.0 - cosTheta) / dtheta;
  }

  public static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }
}
