package BobcatLib.Subsystems.Swerve.SimpleSwerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/** Constants */
public class Constants {
  /** Current Mode ( real , sim , replay , etc.) */
  public static final Mode currentMode =
      RobotBase.isSimulation() ? Mode.SIM : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class FieldConstants {
    public static final double fieldLength = 16.541; // meters
    public static final double fieldWidth = 8.211;
    public static final double noteDiameter = Units.inchesToMeters(14);
    public static final double speakerHeight = Units.inchesToMeters(80.4375); // Center of opening
    public static final Translation2d blueSpeakerPose =
        new Translation2d(
            Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42)); // Center of the opening
    public static final Translation2d redSpeakerPose =
        new Translation2d(
            Units.inchesToMeters(652.73 - 12),
            Units.inchesToMeters(218.42)); // Center of the opening //652.73
    public static final Pose2d blueAmpCenter =
        new Pose2d(
            new Translation2d(Units.inchesToMeters(72.455), fieldWidth),
            Rotation2d.fromDegrees(90));
    public static final Pose2d redAmpCenter =
        new Pose2d(
            new Translation2d(fieldLength - Units.inchesToMeters(72.455), fieldWidth),
            Rotation2d.fromDegrees(-90));
  }

  /** Swerve Constants */
  public static final class SwerveConstants {
    public static final boolean firstOrderDriving = true;
    public static final String canivorename = "Practice";
    public static final Matrix<N3, N1> visionStdDevs =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    public static final Matrix<N3, N1> stateStdDevs =
        VecBuilder.fill(01, 0.01, Units.degreesToRadians(1));
  }
}
