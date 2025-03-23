package GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class RotationUtil {

  /**
   * wraps the rotation2d to be within one positive rotation
   *
   * <p>370 degrees becomes 10 degrees -10 degrees becomes 350 degrees
   */
  public static Rotation2d wrapRot2d(Rotation2d rot) {
    return Rotation2d.fromDegrees(((rot.getDegrees() % 360) + 360) % 360);
  }

  public static Rotation3d wrapRot3d(Rotation3d rot) {
    double wrappedPitch = wrapRot2d(new Rotation2d(rot.getY())).getRadians();
    double wrappedRoll = wrapRot2d(new Rotation2d(rot.getX())).getRadians();
    double wrappedYaw = wrapRot2d(new Rotation2d(rot.getZ())).getRadians();

    return new Rotation3d(wrappedRoll, wrappedPitch, wrappedYaw);
  }
}
