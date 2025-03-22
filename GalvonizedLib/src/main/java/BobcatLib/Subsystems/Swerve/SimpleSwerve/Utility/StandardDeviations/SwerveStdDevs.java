package BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.StandardDeviations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SwerveStdDevs {

  public StandardDeviation autoStdDevs;
  public StandardDeviation teleStdDevs;

  public SwerveStdDevs(StandardDeviation autoStdDevs, StandardDeviation teleStdDevs) {
    this.autoStdDevs = autoStdDevs;
    this.teleStdDevs = teleStdDevs;
  }

  @SuppressWarnings("unchecked")
  public Matrix<N3, N1>[] toMatrix() {
    return new Matrix[] {
      autoStdDevs.trustMatrix(),
      teleStdDevs.trustMatrix(),
      autoStdDevs.distrustMatrix(),
      teleStdDevs.distrustMatrix()
    };
  }
}
