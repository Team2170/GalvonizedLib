package GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionObservation {
  Pose2d m_pose;
  double m_timestamp;
  Matrix<N3, N1> m_stdDev;

  public VisionObservation(Pose2d pose, double timestamp, Matrix<N3, N1> stdDev) {
    m_pose = pose;
    m_timestamp = timestamp;
    m_stdDev = stdDev;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getTimestamp() {
    return m_timestamp;
  }

  public Matrix<N3, N1> getStdDev() {
    return m_stdDev;
  }

  public enum LLTYPE {
    LL2,
    LL3,
    LL3G,
    LL4
  }
}
