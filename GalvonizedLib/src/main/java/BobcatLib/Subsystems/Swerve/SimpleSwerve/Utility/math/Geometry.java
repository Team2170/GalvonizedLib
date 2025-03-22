package BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math;

import edu.wpi.first.math.geometry.Rotation2d;

/** Geometry class used for higher order kinementics in swerve. */
public class Geometry {
  /**
   * @param rad
   * @return ange between 0 and 2pi
   */
  public static double get0to2Pi(double rad) {
    rad = rad % (2 * Math.PI);
    if (rad < (0)) {
      rad += (2 * Math.PI);
    } // should this be here?
    return rad;
  }

  /**
   * @param rot
   * @return angle in 0 to 2pi
   */
  public static double get0to2Pi(Rotation2d rot) {
    return get0to2Pi(rot.getRadians());
  }
  /**
   * wraps the rotation2d to be within one rotation, i.e. a rotation2d with a value of 370 degrees
   * will return a rotation2d with a value of 10 degrees
   */
  public static Rotation2d wrapRot2d(Rotation2d rot) {
    return Rotation2d.fromRadians(get0to2Pi(rot));
  }
}
