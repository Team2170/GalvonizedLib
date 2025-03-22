package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface AutomatedSwerve {

  // auto align sets the drivetrain to a specified angle
  // aim assist adds a slight output on top of the drivers control,
  // helping it to drive towards the desired coordinate point

  /**
   * @return the rotation to turn to when rotation assist is active
   */
  public default Rotation2d autoAlignAngle() {
    return new Rotation2d();
  }

  public default void setAutoAlignAngle(Rotation2d angle) {}

  /**
   * @return the translation the aim assist will try to line up with
   */
  public default Translation2d aimAssistTranslation() {
    return new Translation2d();
  }

  public default void setAimAssistTranslation(Translation2d translation) {}
}
