package BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.StandardDeviations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StandardDeviation {
  private double positionTrust;
  private double positionDistrust;
  private double thetaTrust;
  private double thetaDistrust;

  /**
   * standard deviations are how much our vision measurements tend to be off by, or basically how
   * much we trust the vision measurements. Higher numbers mean the measurements are usually off by
   * more, and we trust them less, lower numbers means that the measurements are off by less.
   *
   * <p>there are two standard deviations, with two variants each, these impact vision measurements.
   *
   * <p>
   *
   * <p>Position is your (x,y) coordinates on the field
   *
   * <p>Theta is your heading in radians
   *
   * <p>Trust std devs are used when we have reliable data, distrust are for when we have unreliable
   * data (i.e. the robot can only see one tag)
   *
   * <p>
   *
   * @param posTrust trust position standard deviation,meters
   * @param posDistrust distrust position standard deviation, meters
   * @param thetaTrust trust rotation standard deviation, radians
   * @param thetaDistrust trust rotation standard deviation, radians
   */
  public StandardDeviation(
      double posTrust, double posDistrust, double thetaTrust, double thetaDistrust) {
    positionTrust = posTrust;
    positionDistrust = posDistrust;
    this.thetaTrust = thetaTrust;
    this.thetaDistrust = thetaDistrust;
  }

  public Matrix<N3, N1> trustMatrix() {
    return VecBuilder.fill(positionTrust, positionTrust, thetaTrust);
  }

  public Matrix<N3, N1> distrustMatrix() {
    return VecBuilder.fill(positionDistrust, positionDistrust, thetaDistrust);
  }
}
